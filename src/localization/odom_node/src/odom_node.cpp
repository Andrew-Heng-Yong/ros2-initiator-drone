#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kSmallAngle = 1.0e-8;

tf2::Vector3 apply_deadband(const tf2::Vector3 & value, double threshold)
{
  return tf2::Vector3(
    std::abs(value.x()) < threshold ? 0.0 : value.x(),
    std::abs(value.y()) < threshold ? 0.0 : value.y(),
    std::abs(value.z()) < threshold ? 0.0 : value.z());
}

tf2::Quaternion quaternion_from_rpy(const std::vector<double> & rpy, const std::string & name)
{
  if (rpy.size() != 3U) {
    throw std::invalid_argument(name + " must contain roll, pitch, and yaw");
  }
  tf2::Quaternion quaternion;
  quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
  quaternion.normalize();
  return quaternion;
}

tf2::Quaternion delta_quaternion(const tf2::Vector3 & angular_velocity, double dt)
{
  const double angle = angular_velocity.length() * dt;
  if (angle < kSmallAngle) {
    tf2::Quaternion delta(
      0.5 * angular_velocity.x() * dt,
      0.5 * angular_velocity.y() * dt,
      0.5 * angular_velocity.z() * dt,
      1.0);
    delta.normalize();
    return delta;
  }
  return tf2::Quaternion(angular_velocity.normalized(), angle);
}

bool is_finite(const tf2::Vector3 & value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

geometry_msgs::msg::Quaternion quaternion_message(const tf2::Quaternion & value)
{
  geometry_msgs::msg::Quaternion message;
  message.x = value.x();
  message.y = value.y();
  message.z = value.z();
  message.w = value.w();
  return message;
}

geometry_msgs::msg::Vector3 vector_message(const tf2::Vector3 & value)
{
  geometry_msgs::msg::Vector3 message;
  message.x = value.x();
  message.y = value.y();
  message.z = value.z();
  return message;
}

}  // namespace

class OdomNode final : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odom_node")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    calibrated_imu_topic_ =
      declare_parameter<std::string>("calibrated_imu_topic", "/imu/data_calibrated");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    calibration_status_topic_ =
      declare_parameter<std::string>("calibration_status_topic", "/odom/calibrated");
    calibration_service_name_ =
      declare_parameter<std::string>("calibration_service", "/odom/calibrate");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    static_override_ = declare_parameter<bool>("static_override", false);
    quality_override_ = declare_parameter<bool>("quality_override", false);
    invert_yaw_ = declare_parameter<bool>("invert_yaw", true);

    gyro_deadband_rad_s_ = declare_parameter<double>("gyro_deadband_rad_s", 0.005);
    calibrate_on_startup_ = declare_parameter<bool>("calibrate_on_startup", true);
    initialization_samples_ = declare_parameter<int>("initialization_samples", 200);
    startup_initialization_samples_ =
      declare_parameter<int>("startup_initialization_samples", 1000);
    max_calibration_angular_speed_rad_s_ =
      declare_parameter<double>("max_calibration_angular_speed_rad_s", 0.50);
    max_calibration_gyro_stddev_rad_s_ =
      declare_parameter<double>("max_calibration_gyro_stddev_rad_s", 0.03);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.25);

    imu_to_body_ = quaternion_from_rpy(
      declare_parameter<std::vector<double>>("imu_to_body_rotation_rpy", {0.0, 0.0, 0.0}),
      "imu_to_body_rotation_rpy");

    unobserved_position_variance_ =
      declare_parameter<double>("unobserved_position_variance", 1.0e6);
    static_position_variance_ =
      declare_parameter<double>("static_position_variance", 0.01);
    quality_override_position_variance_ =
      declare_parameter<double>("quality_override_position_variance", 0.01);
    initial_orientation_variance_ =
      declare_parameter<double>("initial_orientation_variance", 0.01);
    angular_velocity_variance_ =
      declare_parameter<double>("angular_velocity_variance", 0.02);

    validate_parameters();
    orientation_variance_ = initial_orientation_variance_;

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    calibrated_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      calibrated_imu_topic_, rclcpp::SensorDataQoS());
    calibration_status_publisher_ = create_publisher<std_msgs::msg::Bool>(
      calibration_status_topic_, rclcpp::QoS(1).reliable().transient_local());
    calibration_status_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_current_calibration_status();});
    if (publish_tf_) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(std::move(message));});
    calibration_service_ = create_service<std_srvs::srv::Trigger>(
      calibration_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        start_calibration(response);
      });

    if (static_override_) {
      initialized_ = true;
      publish_calibration_status(true);
      RCLCPP_WARN(
        get_logger(),
        "Odometry static override active: calibration and gyro integration are disabled; "
        "publishing a fixed calibrated pose at the origin");
    } else if (calibrate_on_startup_) {
      reset_for_calibration(startup_initialization_samples_);
      RCLCPP_INFO(
        get_logger(),
        "Gyro calibration started (%d stationary samples); imu=%s output=%s",
        active_initialization_samples_, imu_topic_.c_str(), odom_topic_.c_str());
    } else {
      initialized_ = true;
      publish_calibration_status(true);
      RCLCPP_WARN(
        get_logger(),
        "Startup gyro calibration is disabled; using zero bias and identity orientation");
    }
    if (quality_override_ && !static_override_) {
      RCLCPP_WARN(
        get_logger(),
        "Odometry quality override active: unobserved translation will be reported with "
        "variance %.6f m^2 so consumers treat the pose as tracked",
        quality_override_position_variance_);
    }
  }

private:
  void validate_parameters() const
  {
    if (imu_topic_.empty() || calibrated_imu_topic_.empty() || odom_topic_.empty() ||
      calibration_status_topic_.empty() || calibration_service_name_.empty() ||
      odom_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("topic, service, and frame parameters must not be empty");
    }
    if (gyro_deadband_rad_s_ < 0.0 || initialization_samples_ < 10 ||
      startup_initialization_samples_ < 10 || max_calibration_angular_speed_rad_s_ <= 0.0 ||
      max_calibration_gyro_stddev_rad_s_ <= 0.0 || max_imu_gap_sec_ <= 0.0 ||
      unobserved_position_variance_ <= 0.0 || static_position_variance_ < 0.0 ||
      quality_override_position_variance_ < 0.0 ||
      initial_orientation_variance_ < 0.0 ||
      angular_velocity_variance_ < 0.0)
    {
      throw std::invalid_argument("invalid gyro odometry parameters");
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & message)
  {
    if (static_override_) {
      publish_outputs(
        rclcpp::Time(message->header.stamp), tf2::Vector3(0.0, 0.0, 0.0), true);
      return;
    }

    const tf2::Vector3 mounted_gyro = tf2::quatRotate(
      imu_to_body_, tf2::Vector3(
        message->angular_velocity.x,
        message->angular_velocity.y,
        message->angular_velocity.z));
    const tf2::Vector3 raw_gyro(
      mounted_gyro.x(), mounted_gyro.y(),
      invert_yaw_ ? -mounted_gyro.z() : mounted_gyro.z());
    if (!is_finite(raw_gyro)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring non-finite gyro sample");
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    if (!initialized_) {
      collect_calibration_sample(raw_gyro, stamp);
      return;
    }

    const tf2::Vector3 angular_velocity =
      apply_deadband(raw_gyro - gyro_bias_, gyro_deadband_rad_s_);
    if (!has_previous_sample_) {
      previous_angular_velocity_ = angular_velocity;
      last_imu_stamp_ = stamp;
      has_previous_sample_ = true;
      publish_outputs(stamp, angular_velocity);
      return;
    }

    const double dt = (stamp - last_imu_stamp_).seconds();
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring out-of-order gyro timestamp");
      return;
    }

    if (dt <= max_imu_gap_sec_) {
      // Angular velocity is expressed in base_link. Right multiplication integrates this
      // body-frame rate into the base_link-to-odom orientation. Averaging adjacent samples
      // avoids the systematic phase error from treating every newest sample as constant.
      const tf2::Vector3 midpoint_angular_velocity =
        (previous_angular_velocity_ + angular_velocity) * 0.5;
      orientation_ = orientation_ * delta_quaternion(midpoint_angular_velocity, dt);
      orientation_.normalize();
      orientation_variance_ += angular_velocity_variance_ * dt * dt;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Gyro gap %.3f s exceeds %.3f s; not integrating across the gap",
        dt, max_imu_gap_sec_);
    }

    previous_angular_velocity_ = angular_velocity;
    last_imu_stamp_ = stamp;
    publish_outputs(stamp, angular_velocity);
  }

  void start_calibration(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (static_override_) {
      response->success = false;
      response->message = "calibration is disabled while the odometry static override is active";
      return;
    }
    reset_for_calibration(initialization_samples_);
    response->success = true;
    response->message = "gyro calibration started; keep the drone stationary";
    RCLCPP_INFO(
      get_logger(), "Gyro calibration started; collecting %d stationary samples",
      initialization_samples_);
  }

  void reset_for_calibration(int sample_count)
  {
    initialized_ = false;
    has_previous_sample_ = false;
    active_initialization_samples_ = sample_count;
    initialization_count_ = 0;
    gyro_sum_.setValue(0.0, 0.0, 0.0);
    gyro_squared_sum_.setValue(0.0, 0.0, 0.0);
    gyro_bias_.setValue(0.0, 0.0, 0.0);
    previous_angular_velocity_.setValue(0.0, 0.0, 0.0);
    orientation_ = tf2::Quaternion::getIdentity();
    orientation_variance_ = initial_orientation_variance_;
    publish_calibration_status(false);
  }

  void reset_calibration_window()
  {
    initialization_count_ = 0;
    gyro_sum_.setValue(0.0, 0.0, 0.0);
    gyro_squared_sum_.setValue(0.0, 0.0, 0.0);
  }

  void collect_calibration_sample(const tf2::Vector3 & gyro, const rclcpp::Time & stamp)
  {
    gyro_sum_ += gyro;
    gyro_squared_sum_ += tf2::Vector3(
      gyro.x() * gyro.x(), gyro.y() * gyro.y(), gyro.z() * gyro.z());
    ++initialization_count_;
    if (initialization_count_ < active_initialization_samples_) {
      return;
    }

    const double sample_count = static_cast<double>(initialization_count_);
    const tf2::Vector3 mean = gyro_sum_ / sample_count;
    const tf2::Vector3 variance(
      std::max(0.0, gyro_squared_sum_.x() / sample_count - mean.x() * mean.x()),
      std::max(0.0, gyro_squared_sum_.y() / sample_count - mean.y() * mean.y()),
      std::max(0.0, gyro_squared_sum_.z() / sample_count - mean.z() * mean.z()));
    const double maximum_stddev = std::sqrt(std::max({variance.x(), variance.y(), variance.z()}));

    if (maximum_stddev > max_calibration_gyro_stddev_rad_s_) {
      RCLCPP_WARN(
        get_logger(),
        "Gyro moved during calibration (max stddev %.5f rad/s, mean %.5f rad/s); "
        "restarting the stationary sample window",
        maximum_stddev, mean.length());
      reset_calibration_window();
      return;
    }

    // A stationary gyro can have a substantial constant zero-rate offset; that mean is exactly
    // the bias this window is intended to learn. Keep only a generous sanity cap for a bad sensor
    // or a calibration attempted during sustained rotation, and use sample variation to detect
    // ordinary movement.
    if (mean.length() > max_calibration_angular_speed_rad_s_) {
      RCLCPP_WARN(
        get_logger(),
        "Gyro zero-rate bias %.5f rad/s exceeds calibration limit %.5f rad/s "
        "(max stddev %.5f rad/s); restarting the stationary sample window",
        mean.length(), max_calibration_angular_speed_rad_s_, maximum_stddev);
      reset_calibration_window();
      return;
    }

    gyro_bias_ = mean;
    previous_angular_velocity_ = apply_deadband(gyro - gyro_bias_, gyro_deadband_rad_s_);
    orientation_ = tf2::Quaternion::getIdentity();
    orientation_variance_ = initial_orientation_variance_;
    last_imu_stamp_ = stamp;
    has_previous_sample_ = true;
    initialized_ = true;
    publish_calibration_status(true);
    RCLCPP_INFO(
      get_logger(),
      "Gyro initialized after %d samples: bias [%.6f %.6f %.6f] rad/s, "
      "max stddev %.6f rad/s",
      initialization_count_, gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z(), maximum_stddev);
    publish_outputs(stamp, previous_angular_velocity_);
  }

  void publish_calibration_status(bool calibrated)
  {
    calibrated_ = calibrated;
    publish_current_calibration_status();
  }

  void publish_current_calibration_status()
  {
    std_msgs::msg::Bool message;
    message.data = calibrated_;
    calibration_status_publisher_->publish(message);
  }

  void publish_outputs(
    const rclcpp::Time & stamp, const tf2::Vector3 & angular_velocity,
    bool static_pose = false)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = base_frame_;
    imu.orientation = quaternion_message(orientation_);
    imu.angular_velocity = vector_message(angular_velocity);
    imu.orientation_covariance[0] = orientation_variance_;
    imu.orientation_covariance[4] = orientation_variance_;
    imu.orientation_covariance[8] = orientation_variance_;
    imu.angular_velocity_covariance[0] = angular_velocity_variance_;
    imu.angular_velocity_covariance[4] = angular_velocity_variance_;
    imu.angular_velocity_covariance[8] = angular_velocity_variance_;
    // This estimator intentionally does not use or estimate linear acceleration.
    imu.linear_acceleration_covariance[0] = -1.0;
    calibrated_imu_publisher_->publish(imu);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.orientation = quaternion_message(orientation_);
    odometry.twist.twist.angular = vector_message(angular_velocity);

    // In normal gyro-only mode, zero translation is an unobserved placeholder. Static override is
    // an explicit promise that the robot is fixed at the origin. Quality override makes no such
    // promise; it deliberately changes only the reported covariance for visualization clients.
    const double position_variance = static_pose ? static_position_variance_ :
      (quality_override_ ? quality_override_position_variance_ :
      unobserved_position_variance_);
    odometry.pose.covariance[0] = position_variance;
    odometry.pose.covariance[7] = position_variance;
    odometry.pose.covariance[14] = position_variance;
    odometry.pose.covariance[21] = orientation_variance_;
    odometry.pose.covariance[28] = orientation_variance_;
    odometry.pose.covariance[35] = orientation_variance_;
    odometry.twist.covariance[0] = position_variance;
    odometry.twist.covariance[7] = position_variance;
    odometry.twist.covariance[14] = position_variance;
    odometry.twist.covariance[21] = angular_velocity_variance_;
    odometry.twist.covariance[28] = angular_velocity_variance_;
    odometry.twist.covariance[35] = angular_velocity_variance_;
    odometry_publisher_->publish(odometry);

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.rotation = quaternion_message(orientation_);
      transform_broadcaster_->sendTransform(transform);
    }
  }

  std::string imu_topic_;
  std::string calibrated_imu_topic_;
  std::string odom_topic_;
  std::string calibration_status_topic_;
  std::string calibration_service_name_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_ = true;
  bool static_override_ = false;
  bool quality_override_ = false;
  bool invert_yaw_ = true;
  bool calibrate_on_startup_ = true;
  bool calibrated_ = false;
  bool initialized_ = false;
  bool has_previous_sample_ = false;
  int initialization_samples_ = 200;
  int startup_initialization_samples_ = 1000;
  int active_initialization_samples_ = 1000;
  int initialization_count_ = 0;
  double gyro_deadband_rad_s_ = 0.005;
  double max_calibration_angular_speed_rad_s_ = 0.50;
  double max_calibration_gyro_stddev_rad_s_ = 0.03;
  double max_imu_gap_sec_ = 0.25;
  double unobserved_position_variance_ = 1.0e6;
  double static_position_variance_ = 0.01;
  double quality_override_position_variance_ = 0.01;
  double initial_orientation_variance_ = 0.01;
  double orientation_variance_ = 0.01;
  double angular_velocity_variance_ = 0.02;

  tf2::Quaternion imu_to_body_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion orientation_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 gyro_bias_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_squared_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 previous_angular_velocity_{0.0, 0.0, 0.0};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr calibrated_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_publisher_;
  rclcpp::TimerBase::SharedPtr calibration_status_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OdomNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("odom_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}

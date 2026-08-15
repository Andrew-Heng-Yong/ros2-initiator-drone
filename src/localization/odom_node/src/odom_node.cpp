#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "flow_range_sensor_node/msg/optical_flow.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kSmallAngle = 1.0e-8;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStandardGravity = 9.80665;
constexpr double kMinimumUsableAccelerationMagnitude = 0.25;
constexpr double kMaximumUsableAccelerationMagnitude = 50.0;

tf2::Vector3 apply_deadband(const tf2::Vector3 & value, double threshold)
{
  return tf2::Vector3(
    std::abs(value.x()) < threshold ? 0.0 : value.x(),
    std::abs(value.y()) < threshold ? 0.0 : value.y(),
    std::abs(value.z()) < threshold ? 0.0 : value.z());
}

tf2::Vector3 limit_magnitude(const tf2::Vector3 & value, double maximum)
{
  const double magnitude = value.length();
  if (magnitude <= maximum || magnitude < kSmallAngle) {
    return value;
  }
  return value * (maximum / magnitude);
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

tf2::Quaternion rotation_between_vectors(
  const tf2::Vector3 & source, const tf2::Vector3 & target)
{
  if (source.length() < kSmallAngle || target.length() < kSmallAngle) {
    return tf2::Quaternion::getIdentity();
  }
  const tf2::Vector3 source_unit = source.normalized();
  const tf2::Vector3 target_unit = target.normalized();
  const double dot = std::clamp(source_unit.dot(target_unit), -1.0, 1.0);
  if (dot > 1.0 - kSmallAngle) {
    return tf2::Quaternion::getIdentity();
  }
  if (dot < -1.0 + kSmallAngle) {
    tf2::Vector3 axis = source_unit.cross(tf2::Vector3(1.0, 0.0, 0.0));
    if (axis.length() < kSmallAngle) {
      axis = source_unit.cross(tf2::Vector3(0.0, 1.0, 0.0));
    }
    axis.normalize();
    return tf2::Quaternion(axis, kPi);
  }
  const tf2::Vector3 cross = source_unit.cross(target_unit);
  tf2::Quaternion rotation(cross.x(), cross.y(), cross.z(), 1.0 + dot);
  rotation.normalize();
  return rotation;
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
    flow_topic_ = declare_parameter<std::string>("flow_topic", "/optical_flow/raw");
    range_topic_ = declare_parameter<std::string>("range_topic", "/range/down");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    static_override_ = declare_parameter<bool>("static_override", false);
    quality_override_ = declare_parameter<bool>("quality_override", false);
    integrate_linear_acceleration_ =
      declare_parameter<bool>("integrate_linear_acceleration", true);
    auto_scale_acceleration_ = declare_parameter<bool>("auto_scale_acceleration", true);
    imu_average_window_size_ = declare_parameter<int>("imu_average_window_size", 10);

    gyro_deadband_rad_s_ = declare_parameter<double>("gyro_deadband_rad_s", 0.005);
    initialization_samples_ = declare_parameter<int>("initialization_samples", 200);
    startup_initialization_samples_ =
      declare_parameter<int>("startup_initialization_samples", 1000);
    max_calibration_angular_speed_rad_s_ =
      declare_parameter<double>("max_calibration_angular_speed_rad_s", 0.50);
    max_calibration_gyro_stddev_rad_s_ =
      declare_parameter<double>("max_calibration_gyro_stddev_rad_s", 0.03);
    max_calibration_accel_stddev_m_s2_ =
      declare_parameter<double>("max_calibration_accel_stddev_m_s2", 0.25);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.25);
    acceleration_deadband_m_s2_ =
      declare_parameter<double>("acceleration_deadband_m_s2", 0.03);
    acceleration_filter_time_constant_sec_ =
      declare_parameter<double>("acceleration_filter_time_constant_sec", 0.08);
    velocity_damping_per_sec_ = declare_parameter<double>("velocity_damping_per_sec", 0.05);
    max_linear_acceleration_m_s2_ =
      declare_parameter<double>("max_linear_acceleration_m_s2", 15.0);
    max_linear_speed_m_s_ = declare_parameter<double>("max_linear_speed_m_s", 5.0);

    use_optical_flow_ = declare_parameter<bool>("use_optical_flow", true);
    minimum_flow_quality_ = declare_parameter<int>("minimum_flow_quality", 25);
    maximum_flow_shutter_ = declare_parameter<int>("maximum_flow_shutter", 7999);
    flow_radians_per_count_ = declare_parameter<double>("flow_radians_per_count", 0.0025);
    flow_velocity_gain_ = declare_parameter<double>("flow_velocity_gain", 0.65);
    flow_timeout_sec_ = declare_parameter<double>("flow_timeout_sec", 0.20);
    max_flow_angular_speed_rad_s_ =
      declare_parameter<double>("max_flow_angular_speed_rad_s", 0.50);
    max_flow_linear_speed_m_s_ =
      declare_parameter<double>("max_flow_linear_speed_m_s", 5.0);
    flow_to_body_matrix_ = declare_parameter<std::vector<double>>(
      "flow_to_body_matrix", {0.0, -1.0, 1.0, 0.0});

    use_rangefinder_ = declare_parameter<bool>("use_rangefinder", true);
    range_position_gain_ = declare_parameter<double>("range_position_gain", 0.25);
    range_velocity_gain_ = declare_parameter<double>("range_velocity_gain", 0.20);
    range_timeout_sec_ = declare_parameter<double>("range_timeout_sec", 0.20);
    maximum_range_vertical_speed_m_s_ =
      declare_parameter<double>("maximum_range_vertical_speed_m_s", 3.0);
    maximum_range_innovation_m_ =
      declare_parameter<double>("maximum_range_innovation_m", 0.75);
    minimum_range_vertical_projection_ =
      declare_parameter<double>("minimum_range_vertical_projection", 0.50);
    range_reference_distance_m_ =
      declare_parameter<double>("range_reference_distance_m", -1.0);

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
    linear_acceleration_variance_ =
      declare_parameter<double>("linear_acceleration_variance", 0.10);
    initial_inertial_position_variance_ =
      declare_parameter<double>("initial_inertial_position_variance", 0.25);
    position_variance_growth_per_sec_ =
      declare_parameter<double>("position_variance_growth_per_sec", 0.25);
    flow_velocity_variance_ = declare_parameter<double>("flow_velocity_variance", 0.04);
    range_position_variance_ = declare_parameter<double>("range_position_variance", 0.01);
    range_velocity_variance_ = declare_parameter<double>("range_velocity_variance", 0.04);

    validate_parameters();
    orientation_variance_ = initial_orientation_variance_;
    position_variance_ = initial_inertial_position_variance_;

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
    if (use_optical_flow_) {
      flow_subscription_ =
        create_subscription<flow_range_sensor_node::msg::OpticalFlow>(
        flow_topic_, rclcpp::SensorDataQoS(),
        [this](flow_range_sensor_node::msg::OpticalFlow::ConstSharedPtr message)
        {
          on_optical_flow(std::move(message));
        });
    }
    if (use_rangefinder_) {
      range_subscription_ = create_subscription<sensor_msgs::msg::Range>(
        range_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr message)
        {
          on_range(std::move(message));
        });
    }
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
        "Odometry static override active: calibration and IMU integration are disabled; "
        "publishing a fixed calibrated pose at the origin");
    } else {
      reset_for_calibration(startup_initialization_samples_);
      RCLCPP_INFO(
        get_logger(),
        "IMU calibration started (%d stationary samples); imu=%s output=%s",
        active_initialization_samples_, imu_topic_.c_str(), odom_topic_.c_str());
    }
    if (quality_override_ && !static_override_) {
      RCLCPP_WARN(
        get_logger(),
        "Odometry quality override active: position will be reported with variance %.6f m^2",
        quality_override_position_variance_);
    }
    if (integrate_linear_acceleration_ && !static_override_) {
      RCLCPP_WARN(
        get_logger(),
        "Inertial translation active: acceleration is the fallback between accepted external "
        "flow/range measurements and will drift whenever external aiding is unavailable");
    }
    if ((use_optical_flow_ || use_rangefinder_) && !static_override_) {
      RCLCPP_INFO(
        get_logger(),
        "External translation aiding enabled: flow=%s (%s), range=%s (%s)",
        use_optical_flow_ ? "on" : "off", flow_topic_.c_str(),
        use_rangefinder_ ? "on" : "off", range_topic_.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "IMU rolling average enabled over %d reads", imu_average_window_size_);
  }

private:
  void validate_parameters() const
  {
    if (imu_topic_.empty() || calibrated_imu_topic_.empty() || odom_topic_.empty() ||
      calibration_status_topic_.empty() || calibration_service_name_.empty() ||
      (use_optical_flow_ && flow_topic_.empty()) || (use_rangefinder_ && range_topic_.empty()) ||
      odom_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("topic, service, and frame parameters must not be empty");
    }
    if (gyro_deadband_rad_s_ < 0.0 || imu_average_window_size_ < 1 ||
      initialization_samples_ < 10 ||
      startup_initialization_samples_ < 10 || max_calibration_angular_speed_rad_s_ <= 0.0 ||
      max_calibration_gyro_stddev_rad_s_ <= 0.0 ||
      max_calibration_accel_stddev_m_s2_ <= 0.0 || max_imu_gap_sec_ <= 0.0 ||
      acceleration_deadband_m_s2_ < 0.0 || acceleration_filter_time_constant_sec_ < 0.0 ||
      velocity_damping_per_sec_ < 0.0 ||
      max_linear_acceleration_m_s2_ <= 0.0 || max_linear_speed_m_s_ <= 0.0 ||
      unobserved_position_variance_ <= 0.0 || static_position_variance_ < 0.0 ||
      quality_override_position_variance_ < 0.0 ||
      initial_orientation_variance_ < 0.0 ||
      angular_velocity_variance_ < 0.0 || linear_acceleration_variance_ < 0.0 ||
      initial_inertial_position_variance_ < 0.0 || position_variance_growth_per_sec_ < 0.0 ||
      minimum_flow_quality_ < 0 || minimum_flow_quality_ > 255 ||
      maximum_flow_shutter_ < 0 || maximum_flow_shutter_ > 8191 ||
      flow_radians_per_count_ <= 0.0 || flow_velocity_gain_ < 0.0 ||
      flow_velocity_gain_ > 1.0 || flow_timeout_sec_ <= 0.0 ||
      max_flow_angular_speed_rad_s_ <= 0.0 || max_flow_linear_speed_m_s_ <= 0.0 ||
      flow_to_body_matrix_.size() != 4U || range_position_gain_ < 0.0 ||
      range_position_gain_ > 1.0 || range_velocity_gain_ < 0.0 ||
      range_velocity_gain_ > 1.0 || range_timeout_sec_ <= 0.0 ||
      maximum_range_vertical_speed_m_s_ <= 0.0 ||
      maximum_range_innovation_m_ <= 0.0 ||
      minimum_range_vertical_projection_ <= 0.0 || minimum_range_vertical_projection_ > 1.0 ||
      (range_reference_distance_m_ < 0.0 &&
      std::abs(range_reference_distance_m_ + 1.0) > kSmallAngle) ||
      flow_velocity_variance_ < 0.0 ||
      range_position_variance_ < 0.0 || range_velocity_variance_ < 0.0)
    {
      throw std::invalid_argument("invalid gyro odometry parameters");
    }
  }

  rclcpp::Time message_stamp_or_now(const builtin_interfaces::msg::Time & header_stamp) const
  {
    const rclcpp::Time stamp(header_stamp);
    return stamp.nanoseconds() == 0 ? now() : stamp;
  }

  bool measurement_is_recent(
    const rclcpp::Time & measurement_stamp, const rclcpp::Time & output_stamp,
    double timeout_sec) const
  {
    return measurement_stamp.nanoseconds() != 0 &&
           std::abs((output_stamp - measurement_stamp).seconds()) <= timeout_sec;
  }

  void on_optical_flow(
    const flow_range_sensor_node::msg::OpticalFlow::ConstSharedPtr & message)
  {
    if (!initialized_ || static_override_ || !message->range_valid ||
      !std::isfinite(message->ground_distance) || message->ground_distance <= 0.0F ||
      !std::isfinite(message->integration_time) || message->integration_time <= 0.0F ||
      message->integration_time > flow_timeout_sec_ ||
      message->quality < minimum_flow_quality_ || message->shutter > maximum_flow_shutter_ ||
      latest_angular_velocity_.length() > max_flow_angular_speed_rad_s_)
    {
      return;
    }

    const rclcpp::Time stamp = message_stamp_or_now(message->header.stamp);
    if (std::abs((now() - stamp).seconds()) > flow_timeout_sec_) {
      return;
    }

    const double count_to_velocity =
      flow_radians_per_count_ * static_cast<double>(message->ground_distance) /
      static_cast<double>(message->integration_time);
    const double delta_x = message->motion_detected ?
      static_cast<double>(message->delta_x) : 0.0;
    const double delta_y = message->motion_detected ?
      static_cast<double>(message->delta_y) : 0.0;
    const tf2::Vector3 velocity_body(
      count_to_velocity *
      (flow_to_body_matrix_[0] * delta_x + flow_to_body_matrix_[1] * delta_y),
      count_to_velocity *
      (flow_to_body_matrix_[2] * delta_x + flow_to_body_matrix_[3] * delta_y),
      0.0);
    if (!is_finite(velocity_body) || velocity_body.length() > max_flow_linear_speed_m_s_) {
      return;
    }

    const tf2::Vector3 velocity_odom = tf2::quatRotate(orientation_, velocity_body);
    linear_velocity_.setX(
      linear_velocity_.x() * (1.0 - flow_velocity_gain_) +
      velocity_odom.x() * flow_velocity_gain_);
    linear_velocity_.setY(
      linear_velocity_.y() * (1.0 - flow_velocity_gain_) +
      velocity_odom.y() * flow_velocity_gain_);
    last_valid_flow_stamp_ = stamp;
    if (accepted_flow_samples_ == 0) {
      RCLCPP_INFO(
        get_logger(), "Accepted first optical-flow aid: quality=%u range=%.3f m",
        static_cast<unsigned int>(message->quality), message->ground_distance);
    }
    ++accepted_flow_samples_;
  }

  void on_range(const sensor_msgs::msg::Range::ConstSharedPtr & message)
  {
    if (!initialized_ || static_override_ || !std::isfinite(message->range) ||
      message->range < message->min_range || message->range > message->max_range)
    {
      return;
    }

    const rclcpp::Time stamp = message_stamp_or_now(message->header.stamp);
    if (std::abs((now() - stamp).seconds()) > range_timeout_sec_) {
      return;
    }

    const tf2::Vector3 range_ray_odom =
      tf2::quatRotate(orientation_, tf2::Vector3(0.0, 0.0, -1.0));
    const double vertical_projection = std::max(0.0, -range_ray_odom.z());
    if (vertical_projection < minimum_range_vertical_projection_) {
      return;
    }
    const double vertical_height = static_cast<double>(message->range) * vertical_projection;

    if (!range_reference_initialized_) {
      range_reference_height_m_ = range_reference_distance_m_ >= 0.0 ?
        range_reference_distance_m_ : vertical_height;
      previous_range_position_m_ = vertical_height - range_reference_height_m_;
      last_range_measurement_stamp_ = stamp;
      last_valid_range_stamp_ = stamp;
      range_reference_initialized_ = true;
      RCLCPP_INFO(
        get_logger(), "Range reference initialized at %.3f m vertical height",
        range_reference_height_m_);
      ++accepted_range_samples_;
      return;
    }

    const double dt = (stamp - last_range_measurement_stamp_).seconds();
    if (dt <= 0.0 || dt > range_timeout_sec_) {
      previous_range_position_m_ = vertical_height - range_reference_height_m_;
      last_range_measurement_stamp_ = stamp;
      return;
    }

    const double measured_position_z = vertical_height - range_reference_height_m_;
    if (std::abs(measured_position_z - position_.z()) > maximum_range_innovation_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting range innovation %.3f m larger than %.3f m",
        measured_position_z - position_.z(), maximum_range_innovation_m_);
      previous_range_position_m_ = measured_position_z;
      last_range_measurement_stamp_ = stamp;
      return;
    }

    const double measured_velocity_z = std::clamp(
      (measured_position_z - previous_range_position_m_) / dt,
      -maximum_range_vertical_speed_m_s_, maximum_range_vertical_speed_m_s_);
    position_.setZ(
      position_.z() * (1.0 - range_position_gain_) +
      measured_position_z * range_position_gain_);
    linear_velocity_.setZ(
      linear_velocity_.z() * (1.0 - range_velocity_gain_) +
      measured_velocity_z * range_velocity_gain_);

    previous_range_position_m_ = measured_position_z;
    last_range_measurement_stamp_ = stamp;
    last_valid_range_stamp_ = stamp;
    ++accepted_range_samples_;
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & message)
  {
    const tf2::Vector3 mounted_acceleration = tf2::quatRotate(
      imu_to_body_, tf2::Vector3(
        message->linear_acceleration.x,
        message->linear_acceleration.y,
        message->linear_acceleration.z));
    if (!is_finite(mounted_acceleration)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring non-finite accelerometer sample");
      return;
    }

    const tf2::Vector3 raw_gyro = tf2::quatRotate(
      imu_to_body_, tf2::Vector3(
        message->angular_velocity.x,
        message->angular_velocity.y,
        message->angular_velocity.z));
    if (!is_finite(raw_gyro)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring non-finite gyro sample");
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    tf2::Vector3 averaged_acceleration;
    tf2::Vector3 averaged_gyro;
    if (!update_imu_average(
        mounted_acceleration, raw_gyro, averaged_acceleration, averaged_gyro))
    {
      return;
    }

    if (static_override_) {
      publish_outputs(
        stamp, tf2::Vector3(0.0, 0.0, 0.0), averaged_acceleration, true);
      return;
    }

    if (!initialized_) {
      collect_calibration_sample(averaged_gyro, averaged_acceleration, stamp);
      return;
    }

    const tf2::Vector3 angular_velocity =
      apply_deadband(
        tf2::quatRotate(calibration_alignment_, averaged_gyro - gyro_bias_),
        gyro_deadband_rad_s_);
    const tf2::Vector3 calibrated_acceleration =
      tf2::quatRotate(calibration_alignment_, averaged_acceleration) *
      acceleration_scale_factor_;
    latest_angular_velocity_ = angular_velocity;
    if (!has_previous_sample_) {
      previous_angular_velocity_ = angular_velocity;
      last_imu_stamp_ = stamp;
      has_previous_sample_ = true;
      initialize_gravity_reference(calibrated_acceleration);
      publish_outputs(stamp, angular_velocity, calibrated_acceleration);
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
      integrate_translation(calibrated_acceleration, dt);
    } else {
      linear_velocity_.setValue(0.0, 0.0, 0.0);
      filtered_translation_acceleration_.setValue(0.0, 0.0, 0.0);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU gap %.3f s exceeds %.3f s; zeroing velocity and not integrating across the gap",
        dt, max_imu_gap_sec_);
    }

    previous_angular_velocity_ = angular_velocity;
    last_imu_stamp_ = stamp;
    publish_outputs(stamp, angular_velocity, calibrated_acceleration);
  }

  bool update_imu_average(
    const tf2::Vector3 & acceleration, const tf2::Vector3 & gyro,
    tf2::Vector3 & averaged_acceleration, tf2::Vector3 & averaged_gyro)
  {
    acceleration_average_window_.push_back(acceleration);
    gyro_average_window_.push_back(gyro);
    acceleration_average_sum_ += acceleration;
    gyro_average_sum_ += gyro;

    const auto window_size = static_cast<std::size_t>(imu_average_window_size_);
    while (acceleration_average_window_.size() > window_size) {
      acceleration_average_sum_ -= acceleration_average_window_.front();
      acceleration_average_window_.pop_front();
      gyro_average_sum_ -= gyro_average_window_.front();
      gyro_average_window_.pop_front();
    }
    if (acceleration_average_window_.size() < window_size) {
      return false;
    }

    const double sample_count = static_cast<double>(window_size);
    averaged_acceleration = acceleration_average_sum_ / sample_count;
    averaged_gyro = gyro_average_sum_ / sample_count;
    return true;
  }

  void clear_imu_average()
  {
    acceleration_average_window_.clear();
    gyro_average_window_.clear();
    acceleration_average_sum_.setValue(0.0, 0.0, 0.0);
    gyro_average_sum_.setValue(0.0, 0.0, 0.0);
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
    response->message = "IMU calibration started; keep the drone stationary";
    RCLCPP_INFO(
      get_logger(), "IMU calibration started; collecting %d stationary samples",
      initialization_samples_);
  }

  void reset_for_calibration(int sample_count)
  {
    clear_imu_average();
    initialized_ = false;
    has_previous_sample_ = false;
    active_initialization_samples_ = sample_count;
    initialization_count_ = 0;
    gyro_sum_.setValue(0.0, 0.0, 0.0);
    gyro_squared_sum_.setValue(0.0, 0.0, 0.0);
    acceleration_sum_.setValue(0.0, 0.0, 0.0);
    acceleration_squared_sum_.setValue(0.0, 0.0, 0.0);
    gyro_bias_.setValue(0.0, 0.0, 0.0);
    previous_angular_velocity_.setValue(0.0, 0.0, 0.0);
    position_.setValue(0.0, 0.0, 0.0);
    linear_velocity_.setValue(0.0, 0.0, 0.0);
    latest_angular_velocity_.setValue(0.0, 0.0, 0.0);
    filtered_translation_acceleration_.setValue(0.0, 0.0, 0.0);
    gravity_odom_.setValue(0.0, 0.0, 0.0);
    has_gravity_reference_ = false;
    acceleration_integration_available_ = integrate_linear_acceleration_;
    acceleration_scale_factor_ = 1.0;
    calibration_alignment_ = tf2::Quaternion::getIdentity();
    orientation_ = tf2::Quaternion::getIdentity();
    orientation_variance_ = initial_orientation_variance_;
    position_variance_ = initial_inertial_position_variance_;
    range_reference_initialized_ = false;
    range_reference_height_m_ = 0.0;
    previous_range_position_m_ = 0.0;
    last_valid_flow_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_valid_range_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_range_measurement_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    accepted_flow_samples_ = 0;
    accepted_range_samples_ = 0;
    publish_calibration_status(false);
  }

  void reset_calibration_window()
  {
    initialization_count_ = 0;
    gyro_sum_.setValue(0.0, 0.0, 0.0);
    gyro_squared_sum_.setValue(0.0, 0.0, 0.0);
    acceleration_sum_.setValue(0.0, 0.0, 0.0);
    acceleration_squared_sum_.setValue(0.0, 0.0, 0.0);
  }

  void collect_calibration_sample(
    const tf2::Vector3 & gyro, const tf2::Vector3 & linear_acceleration,
    const rclcpp::Time & stamp)
  {
    gyro_sum_ += gyro;
    gyro_squared_sum_ += tf2::Vector3(
      gyro.x() * gyro.x(), gyro.y() * gyro.y(), gyro.z() * gyro.z());
    acceleration_sum_ += linear_acceleration;
    acceleration_squared_sum_ += tf2::Vector3(
      linear_acceleration.x() * linear_acceleration.x(),
      linear_acceleration.y() * linear_acceleration.y(),
      linear_acceleration.z() * linear_acceleration.z());
    ++initialization_count_;
    if (initialization_count_ < active_initialization_samples_) {
      return;
    }

    const double sample_count = static_cast<double>(initialization_count_);
    const tf2::Vector3 gyro_mean = gyro_sum_ / sample_count;
    const tf2::Vector3 gyro_variance(
      std::max(0.0, gyro_squared_sum_.x() / sample_count - gyro_mean.x() * gyro_mean.x()),
      std::max(0.0, gyro_squared_sum_.y() / sample_count - gyro_mean.y() * gyro_mean.y()),
      std::max(0.0, gyro_squared_sum_.z() / sample_count - gyro_mean.z() * gyro_mean.z()));
    const double maximum_gyro_stddev =
      std::sqrt(std::max({gyro_variance.x(), gyro_variance.y(), gyro_variance.z()}));
    const tf2::Vector3 acceleration_mean = acceleration_sum_ / sample_count;
    const tf2::Vector3 acceleration_variance(
      std::max(
        0.0, acceleration_squared_sum_.x() / sample_count -
        acceleration_mean.x() * acceleration_mean.x()),
      std::max(
        0.0, acceleration_squared_sum_.y() / sample_count -
        acceleration_mean.y() * acceleration_mean.y()),
      std::max(
        0.0, acceleration_squared_sum_.z() / sample_count -
        acceleration_mean.z() * acceleration_mean.z()));
    const double maximum_acceleration_stddev = std::sqrt(std::max(
      {acceleration_variance.x(), acceleration_variance.y(), acceleration_variance.z()}));

    if (maximum_gyro_stddev > max_calibration_gyro_stddev_rad_s_) {
      RCLCPP_WARN(
        get_logger(),
        "IMU moved during calibration (gyro stddev %.5f rad/s); "
        "restarting the stationary sample window",
        maximum_gyro_stddev);
      reset_calibration_window();
      return;
    }

    // Accelerometer vibration must not prevent orientation odometry from ever starting. It makes
    // the gravity estimate less trustworthy, but averaging the full window is still preferable to
    // repeatedly discarding an otherwise stationary gyro calibration.
    if (maximum_acceleration_stddev > max_calibration_accel_stddev_m_s2_) {
      RCLCPP_WARN(
        get_logger(),
        "Accelerometer was noisy during calibration (stddev %.5f m/s^2, warning limit "
        "%.5f m/s^2); accepting the averaged gravity reference",
        maximum_acceleration_stddev, max_calibration_accel_stddev_m_s2_);
    }

    // A stationary gyro can have a substantial constant zero-rate offset; that mean is exactly
    // the bias this window is intended to learn. Keep only a generous sanity cap for a bad sensor
    // or a calibration attempted during sustained rotation, and use sample variation to detect
    // ordinary movement.
    if (gyro_mean.length() > max_calibration_angular_speed_rad_s_) {
      RCLCPP_WARN(
        get_logger(),
        "Gyro zero-rate bias %.5f rad/s exceeds calibration limit %.5f rad/s "
        "(max stddev %.5f rad/s); restarting the stationary sample window",
        gyro_mean.length(), max_calibration_angular_speed_rad_s_, maximum_gyro_stddev);
      reset_calibration_window();
      return;
    }

    const double measured_acceleration_magnitude = acceleration_mean.length();
    if (measured_acceleration_magnitude < kMinimumUsableAccelerationMagnitude ||
      measured_acceleration_magnitude > kMaximumUsableAccelerationMagnitude)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Acceleration magnitude %.4f m/s^2 is outside the gravity sanity range; "
        "disabling inertial position integration but continuing orientation odometry",
        measured_acceleration_magnitude);
      acceleration_integration_available_ = false;
      acceleration_scale_factor_ = 1.0;
      gravity_odom_.setValue(0.0, 0.0, 0.0);
      has_gravity_reference_ = false;
    } else {
      acceleration_integration_available_ = integrate_linear_acceleration_;
      acceleration_scale_factor_ = auto_scale_acceleration_ ?
        kStandardGravity / measured_acceleration_magnitude : 1.0;
      const double calibrated_gravity_magnitude =
        measured_acceleration_magnitude * acceleration_scale_factor_;
      // An accelerometer measures specific force, not gravity: at rest it reads +g along the
      // frame's up axis (REP-145), so a levelled base_link reads [0, 0, +g]. Levelling onto
      // -Z instead turns every normally mounted IMU into a 180 degree rotation about Y, which
      // silently mirrors two of the three gyro axes. This is also the value
      // `initialize_gravity_reference` stores, and the two must agree.
      const tf2::Vector3 stationary_specific_force(0.0, 0.0, calibrated_gravity_magnitude);
      calibration_alignment_ =
        rotation_between_vectors(acceleration_mean, stationary_specific_force);
      gravity_odom_ = stationary_specific_force;
      has_gravity_reference_ = true;
      if (auto_scale_acceleration_ &&
        std::abs(acceleration_scale_factor_ - 1.0) > 0.05)
      {
        RCLCPP_WARN(
          get_logger(),
          "Stationary acceleration measured %.4f m/s^2; applying scale %.5f so gravity "
          "and translation use SI acceleration",
          measured_acceleration_magnitude, acceleration_scale_factor_);
      }
    }

    gyro_bias_ = gyro_mean;
    previous_angular_velocity_ = apply_deadband(
      tf2::quatRotate(calibration_alignment_, gyro - gyro_bias_), gyro_deadband_rad_s_);
    orientation_ = tf2::Quaternion::getIdentity();
    orientation_variance_ = initial_orientation_variance_;
    last_imu_stamp_ = stamp;
    has_previous_sample_ = true;
    initialized_ = true;
    publish_calibration_status(true);
    RCLCPP_INFO(
      get_logger(),
      "IMU initialized after %d samples: gyro bias [%.6f %.6f %.6f] rad/s, "
      "gravity reference [%.4f %.4f %.4f] m/s^2, gyro stddev %.6f rad/s, "
      "accel stddev %.6f m/s^2",
      initialization_count_, gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z(),
      gravity_odom_.x(), gravity_odom_.y(), gravity_odom_.z(), maximum_gyro_stddev,
      maximum_acceleration_stddev);
    publish_outputs(
      stamp, previous_angular_velocity_,
      tf2::quatRotate(calibration_alignment_, linear_acceleration) *
      acceleration_scale_factor_);
  }

  void initialize_gravity_reference(const tf2::Vector3 & linear_acceleration)
  {
    if (has_gravity_reference_ || !integrate_linear_acceleration_ ||
      !acceleration_integration_available_)
    {
      return;
    }
    gravity_odom_ = tf2::quatRotate(orientation_, linear_acceleration);
    has_gravity_reference_ = linear_acceleration.length() > kSmallAngle;
  }

  void integrate_translation(const tf2::Vector3 & linear_acceleration, double dt)
  {
    if (!integrate_linear_acceleration_ || !acceleration_integration_available_) {
      if (measurement_is_recent(last_valid_flow_stamp_, now(), flow_timeout_sec_)) {
        position_ += tf2::Vector3(linear_velocity_.x(), linear_velocity_.y(), 0.0) * dt;
        position_variance_ += position_variance_growth_per_sec_ * dt;
      }
      return;
    }
    initialize_gravity_reference(linear_acceleration);
    if (!has_gravity_reference_) {
      return;
    }

    const tf2::Vector3 observed_gravity_and_acceleration =
      tf2::quatRotate(orientation_, linear_acceleration);
    tf2::Vector3 translation_acceleration =
      observed_gravity_and_acceleration - gravity_odom_;
    const double acceleration_filter_alpha = acceleration_filter_time_constant_sec_ < kSmallAngle ?
      1.0 : dt / (acceleration_filter_time_constant_sec_ + dt);
    filtered_translation_acceleration_ =
      filtered_translation_acceleration_ * (1.0 - acceleration_filter_alpha) +
      translation_acceleration * acceleration_filter_alpha;
    translation_acceleration = filtered_translation_acceleration_;

    tf2::Vector3 acceleration_odom = apply_deadband(
      translation_acceleration, acceleration_deadband_m_s2_);
    acceleration_odom = limit_magnitude(acceleration_odom, max_linear_acceleration_m_s2_);
    const tf2::Vector3 previous_velocity = linear_velocity_;
    linear_velocity_ += acceleration_odom * dt;
    const double damping = std::exp(-velocity_damping_per_sec_ * dt);
    linear_velocity_ *= damping;
    linear_velocity_ = limit_magnitude(linear_velocity_, max_linear_speed_m_s_);
    position_ += (previous_velocity + linear_velocity_) * (0.5 * dt);
    position_variance_ += position_variance_growth_per_sec_ * dt;
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
    const tf2::Vector3 & linear_acceleration,
    bool static_pose = false)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = base_frame_;
    imu.orientation = quaternion_message(orientation_);
    imu.angular_velocity = vector_message(angular_velocity);
    imu.linear_acceleration = vector_message(linear_acceleration);
    imu.orientation_covariance[0] = orientation_variance_;
    imu.orientation_covariance[4] = orientation_variance_;
    imu.orientation_covariance[8] = orientation_variance_;
    imu.angular_velocity_covariance[0] = angular_velocity_variance_;
    imu.angular_velocity_covariance[4] = angular_velocity_variance_;
    imu.angular_velocity_covariance[8] = angular_velocity_variance_;
    imu.linear_acceleration_covariance[0] = linear_acceleration_variance_;
    imu.linear_acceleration_covariance[4] = linear_acceleration_variance_;
    imu.linear_acceleration_covariance[8] = linear_acceleration_variance_;
    calibrated_imu_publisher_->publish(imu);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = position_.x();
    odometry.pose.pose.position.y = position_.y();
    odometry.pose.pose.position.z = position_.z();
    odometry.pose.pose.orientation = quaternion_message(orientation_);
    odometry.twist.twist.linear = vector_message(linear_velocity_);
    odometry.twist.twist.angular = vector_message(angular_velocity);

    const bool flow_recent = use_optical_flow_ &&
      measurement_is_recent(last_valid_flow_stamp_, stamp, flow_timeout_sec_);
    const bool range_recent = use_rangefinder_ &&
      measurement_is_recent(last_valid_range_stamp_, stamp, range_timeout_sec_);
    const bool inertial_translation =
      integrate_linear_acceleration_ && acceleration_integration_available_;
    const double horizontal_position_variance = static_pose ? static_position_variance_ :
      (quality_override_ ? quality_override_position_variance_ :
      (inertial_translation || flow_recent ? position_variance_ : unobserved_position_variance_));
    const double vertical_position_variance = static_pose ? static_position_variance_ :
      (quality_override_ ? quality_override_position_variance_ :
      (range_recent ? range_position_variance_ :
      (inertial_translation ? position_variance_ : unobserved_position_variance_)));
    odometry.pose.covariance[0] = horizontal_position_variance;
    odometry.pose.covariance[7] = horizontal_position_variance;
    odometry.pose.covariance[14] = vertical_position_variance;
    odometry.pose.covariance[21] = orientation_variance_;
    odometry.pose.covariance[28] = orientation_variance_;
    odometry.pose.covariance[35] = orientation_variance_;
    odometry.twist.covariance[0] = flow_recent ?
      flow_velocity_variance_ : horizontal_position_variance;
    odometry.twist.covariance[7] = flow_recent ?
      flow_velocity_variance_ : horizontal_position_variance;
    odometry.twist.covariance[14] = range_recent ?
      range_velocity_variance_ : vertical_position_variance;
    odometry.twist.covariance[21] = angular_velocity_variance_;
    odometry.twist.covariance[28] = angular_velocity_variance_;
    odometry.twist.covariance[35] = angular_velocity_variance_;
    odometry_publisher_->publish(odometry);

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = position_.x();
      transform.transform.translation.y = position_.y();
      transform.transform.translation.z = position_.z();
      transform.transform.rotation = quaternion_message(orientation_);
      transform_broadcaster_->sendTransform(transform);
    }
  }

  std::string imu_topic_;
  std::string calibrated_imu_topic_;
  std::string odom_topic_;
  std::string calibration_status_topic_;
  std::string calibration_service_name_;
  std::string flow_topic_;
  std::string range_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_ = true;
  bool static_override_ = false;
  bool quality_override_ = false;
  bool integrate_linear_acceleration_ = true;
  bool auto_scale_acceleration_ = true;
  bool use_optical_flow_ = true;
  bool use_rangefinder_ = true;
  bool calibrated_ = false;
  bool initialized_ = false;
  bool has_previous_sample_ = false;
  bool has_gravity_reference_ = false;
  bool acceleration_integration_available_ = true;
  int initialization_samples_ = 200;
  int startup_initialization_samples_ = 1000;
  int active_initialization_samples_ = 1000;
  int initialization_count_ = 0;
  int imu_average_window_size_ = 10;
  double gyro_deadband_rad_s_ = 0.005;
  double max_calibration_angular_speed_rad_s_ = 0.50;
  double max_calibration_gyro_stddev_rad_s_ = 0.03;
  double max_calibration_accel_stddev_m_s2_ = 0.25;
  double max_imu_gap_sec_ = 0.25;
  double acceleration_deadband_m_s2_ = 0.03;
  double acceleration_filter_time_constant_sec_ = 0.08;
  double velocity_damping_per_sec_ = 0.05;
  double max_linear_acceleration_m_s2_ = 15.0;
  double max_linear_speed_m_s_ = 5.0;
  int minimum_flow_quality_ = 25;
  int maximum_flow_shutter_ = 7999;
  double flow_radians_per_count_ = 0.0025;
  double flow_velocity_gain_ = 0.65;
  double flow_timeout_sec_ = 0.20;
  double max_flow_angular_speed_rad_s_ = 0.50;
  double max_flow_linear_speed_m_s_ = 5.0;
  std::vector<double> flow_to_body_matrix_{0.0, -1.0, 1.0, 0.0};
  double range_position_gain_ = 0.25;
  double range_velocity_gain_ = 0.20;
  double range_timeout_sec_ = 0.20;
  double maximum_range_vertical_speed_m_s_ = 3.0;
  double maximum_range_innovation_m_ = 0.75;
  double minimum_range_vertical_projection_ = 0.50;
  double range_reference_distance_m_ = -1.0;
  double acceleration_scale_factor_ = 1.0;
  double unobserved_position_variance_ = 1.0e6;
  double static_position_variance_ = 0.01;
  double quality_override_position_variance_ = 0.01;
  double initial_orientation_variance_ = 0.01;
  double orientation_variance_ = 0.01;
  double angular_velocity_variance_ = 0.02;
  double linear_acceleration_variance_ = 0.10;
  double initial_inertial_position_variance_ = 0.25;
  double position_variance_growth_per_sec_ = 0.25;
  double position_variance_ = 0.25;
  double flow_velocity_variance_ = 0.04;
  double range_position_variance_ = 0.01;
  double range_velocity_variance_ = 0.04;
  bool range_reference_initialized_ = false;
  double range_reference_height_m_ = 0.0;
  double previous_range_position_m_ = 0.0;
  uint64_t accepted_flow_samples_ = 0;
  uint64_t accepted_range_samples_ = 0;

  tf2::Quaternion imu_to_body_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion calibration_alignment_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion orientation_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 gyro_bias_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_squared_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 acceleration_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 acceleration_squared_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 gravity_odom_{0.0, 0.0, 0.0};
  tf2::Vector3 filtered_translation_acceleration_{0.0, 0.0, 0.0};
  tf2::Vector3 position_{0.0, 0.0, 0.0};
  tf2::Vector3 linear_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 previous_angular_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 latest_angular_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 acceleration_average_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_average_sum_{0.0, 0.0, 0.0};
  std::deque<tf2::Vector3> acceleration_average_window_;
  std::deque<tf2::Vector3> gyro_average_window_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_flow_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_range_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_range_measurement_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr calibrated_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_publisher_;
  rclcpp::TimerBase::SharedPtr calibration_status_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<flow_range_sensor_node::msg::OpticalFlow>::SharedPtr flow_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
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

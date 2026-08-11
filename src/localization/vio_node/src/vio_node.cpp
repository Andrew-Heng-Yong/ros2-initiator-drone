#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kSmall = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

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

tf2::Quaternion shortest_arc(const tf2::Vector3 & from, const tf2::Vector3 & to)
{
  const tf2::Vector3 a = from.normalized();
  const tf2::Vector3 b = to.normalized();
  const double dot = std::clamp(a.dot(b), -1.0, 1.0);
  if (dot > 1.0 - 1.0e-8) {
    return tf2::Quaternion::getIdentity();
  }
  if (dot < -1.0 + 1.0e-8) {
    tf2::Vector3 axis = a.cross(tf2::Vector3(1.0, 0.0, 0.0));
    if (axis.length2() < kSmall) {
      axis = a.cross(tf2::Vector3(0.0, 1.0, 0.0));
    }
    axis.normalize();
    return tf2::Quaternion(axis, kPi);
  }
  const tf2::Vector3 cross = a.cross(b);
  tf2::Quaternion result(cross.x(), cross.y(), cross.z(), 1.0 + dot);
  result.normalize();
  return result;
}

tf2::Quaternion delta_quaternion(const tf2::Vector3 & angular_velocity, double dt)
{
  const double speed = angular_velocity.length();
  if (speed * dt < 1.0e-8) {
    tf2::Quaternion result(
      0.5 * angular_velocity.x() * dt,
      0.5 * angular_velocity.y() * dt,
      0.5 * angular_velocity.z() * dt,
      1.0);
    result.normalize();
    return result;
  }
  return tf2::Quaternion(angular_velocity / speed, speed * dt);
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

geometry_msgs::msg::Point point_message(const tf2::Vector3 & value)
{
  geometry_msgs::msg::Point message;
  message.x = value.x();
  message.y = value.y();
  message.z = value.z();
  return message;
}

}  // namespace

class VioNode final : public rclcpp::Node
{
public:
  VioNode()
  : Node("vio_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    calibrated_imu_topic_ =
      declare_parameter<std::string>("calibrated_imu_topic", "/imu/data_calibrated");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/vio/odometry");
    video_status_topic_ =
      declare_parameter<std::string>("video_status_topic", "/vio/video_working");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    fx_ = declare_parameter<double>("fx", 0.0);
    fy_ = declare_parameter<double>("fy", 0.0);
    cx_ = declare_parameter<double>("cx", 0.0);
    cy_ = declare_parameter<double>("cy", 0.0);
    has_intrinsics_ = fx_ > 0.0 && fy_ > 0.0;

    max_features_ = declare_parameter<int>("max_features", 300);
    min_tracked_features_ = declare_parameter<int>("min_tracked_features", 35);
    feature_quality_ = declare_parameter<double>("feature_quality", 0.01);
    feature_min_distance_ = declare_parameter<double>("feature_min_distance", 12.0);
    ransac_probability_ = declare_parameter<double>("ransac_probability", 0.999);
    ransac_threshold_pixels_ = declare_parameter<double>("ransac_threshold_pixels", 1.5);
    max_visual_rotation_rad_ = declare_parameter<double>("max_visual_rotation_rad", 0.7);
    visual_processing_rate_hz_ = declare_parameter<double>("visual_processing_rate_hz", 5.0);
    image_processing_scale_ = declare_parameter<double>("image_processing_scale", 0.5);
    visual_orientation_weight_ =
      clamp01(declare_parameter<double>("visual_orientation_weight", 0.20));
    visual_translation_weight_ =
      clamp01(declare_parameter<double>("visual_translation_weight", 0.25));
    visual_velocity_weight_ =
      clamp01(declare_parameter<double>("visual_velocity_weight", 0.10));
    maximum_visual_translation_m_ =
      declare_parameter<double>("maximum_visual_translation_m", 2.0);

    gravity_mps2_ = declare_parameter<double>("gravity_mps2", 9.80665);
    calibrated_gyro_deadband_rad_s_ =
      declare_parameter<double>("calibrated_gyro_deadband_rad_s", 0.02);
    calibrated_accel_deadband_mps2_ =
      declare_parameter<double>("calibrated_accel_deadband_mps2", 0.20);
    accelerometer_tilt_correction_weight_ = clamp01(
      declare_parameter<double>("accelerometer_tilt_correction_weight", 0.02));
    accelerometer_gravity_tolerance_mps2_ =
      declare_parameter<double>("accelerometer_gravity_tolerance_mps2", 1.5);
    calibrate_on_startup_ = declare_parameter<bool>("calibrate_on_startup", true);
    initialization_samples_ = declare_parameter<int>("initialization_samples", 20);
    startup_initialization_samples_ =
      declare_parameter<int>("startup_initialization_samples", 1000);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.25);
    max_image_gap_sec_ = declare_parameter<double>("max_image_gap_sec", 1.0);

    camera_to_body_ = quaternion_from_rpy(
      declare_parameter<std::vector<double>>(
        "camera_to_body_rotation_rpy", {-kHalfPi, 0.0, -kHalfPi}),
      "camera_to_body_rotation_rpy");
    imu_to_body_ = quaternion_from_rpy(
      declare_parameter<std::vector<double>>("imu_to_body_rotation_rpy", {0.0, 0.0, 0.0}),
      "imu_to_body_rotation_rpy");

    position_variance_ = declare_parameter<double>("position_variance", 0.05);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.02);
    linear_velocity_variance_ = declare_parameter<double>("linear_velocity_variance", 0.10);
    angular_velocity_variance_ = declare_parameter<double>("angular_velocity_variance", 0.02);

    validate_parameters();

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    calibrated_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      calibrated_imu_topic_, rclcpp::SensorDataQoS());
    calibration_status_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/vio/calibrated", rclcpp::QoS(1).reliable().transient_local());
    visual_tracking_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/vio/visual_tracking", rclcpp::QoS(1).reliable().transient_local());
    video_status_publisher_ = create_publisher<std_msgs::msg::Bool>(
      video_status_topic_, rclcpp::QoS(1).reliable());
    if (publish_tf_) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(std::move(message));});
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {on_image(std::move(message));});
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        on_camera_info(std::move(message));
      });
    calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "/vio/calibrate",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        start_full_calibration(response);
      });

    if (calibrate_on_startup_) {
      reset_estimator_for_calibration(startup_initialization_samples_);
      RCLCPP_INFO(
        get_logger(),
        "Automatic VIO calibration started (%d stationary samples); image=%s imu=%s output=%s",
        active_initialization_samples_, image_topic_.c_str(), imu_topic_.c_str(),
        odom_topic_.c_str());
    } else {
      initialized_ = true;
      publish_calibration_status(true);
      publish_visual_tracking(false);
      RCLCPP_WARN(
        get_logger(),
        "VIO startup calibration is disabled; using zero biases and identity orientation");
    }
  }

private:
  void validate_parameters() const
  {
    if (image_topic_.empty() || imu_topic_.empty() || calibrated_imu_topic_.empty() ||
      odom_topic_.empty() || video_status_topic_.empty() ||
      odom_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("topic and frame parameters must not be empty");
    }
    if (max_features_ < 20 || min_tracked_features_ < 8 ||
      min_tracked_features_ > max_features_ || feature_quality_ <= 0.0 ||
      feature_quality_ > 1.0 || feature_min_distance_ <= 0.0)
    {
      throw std::invalid_argument("invalid feature tracking parameters");
    }
    if (ransac_probability_ <= 0.0 || ransac_probability_ >= 1.0 ||
      ransac_threshold_pixels_ <= 0.0 || max_visual_rotation_rad_ <= 0.0 ||
      visual_processing_rate_hz_ <= 0.0 || image_processing_scale_ < 0.1 ||
      image_processing_scale_ > 1.0)
    {
      throw std::invalid_argument("invalid visual motion parameters");
    }
    if (gravity_mps2_ <= 0.0 || calibrated_gyro_deadband_rad_s_ < 0.0 ||
      calibrated_accel_deadband_mps2_ < 0.0 || initialization_samples_ < 10 ||
      startup_initialization_samples_ < 10 ||
      accelerometer_gravity_tolerance_mps2_ <= 0.0 ||
      max_imu_gap_sec_ <= 0.0 || max_image_gap_sec_ <= 0.0 ||
      maximum_visual_translation_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid IMU/fusion parameters");
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
  {
    if (message->k[0] <= 0.0 || message->k[4] <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring CameraInfo with invalid focal lengths");
      return;
    }
    const bool first = !has_intrinsics_;
    fx_ = message->k[0];
    fy_ = message->k[4];
    cx_ = message->k[2];
    cy_ = message->k[5];
    distortion_coefficients_ = message->d;
    has_intrinsics_ = true;
    if (first) {
      RCLCPP_INFO(
        get_logger(), "Received camera intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
        fx_, fy_, cx_, cy_);
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & message)
  {
    const tf2::Vector3 raw_gyro = tf2::quatRotate(
      imu_to_body_, tf2::Vector3(
        message->angular_velocity.x, message->angular_velocity.y,
        message->angular_velocity.z));
    const tf2::Vector3 raw_acceleration = tf2::quatRotate(
      imu_to_body_, tf2::Vector3(
        message->linear_acceleration.x, message->linear_acceleration.y,
        message->linear_acceleration.z));
    if (!std::isfinite(raw_gyro.length2()) || !std::isfinite(raw_acceleration.length2()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring non-finite IMU sample");
      return;
    }

    if (!initialized_) {
      collect_initialization_sample(raw_gyro, raw_acceleration, message->header.stamp);
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    const double dt = (stamp - last_imu_stamp_).seconds();
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring out-of-order IMU timestamp");
      return;
    }
    if (dt > max_imu_gap_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "IMU gap %.3f s exceeds %.3f s; skipping propagation across the gap",
        dt, max_imu_gap_sec_);
      last_imu_stamp_ = stamp;
      return;
    }

    const tf2::Vector3 angular_velocity = raw_gyro - gyro_bias_;
    const tf2::Vector3 acceleration_body = raw_acceleration * accel_scale_correction_;
    const tf2::Quaternion old_orientation = orientation_;
    orientation_ = orientation_ * delta_quaternion(angular_velocity, dt);
    orientation_.normalize();

    const double acceleration_magnitude = acceleration_body.length();
    if (std::abs(acceleration_magnitude - gravity_mps2_) <=
      accelerometer_gravity_tolerance_mps2_)
    {
      const tf2::Vector3 measured_gravity_world =
        tf2::quatRotate(orientation_, acceleration_body);
      const tf2::Quaternion full_tilt_correction = shortest_arc(
        measured_gravity_world, tf2::Vector3(0.0, 0.0, gravity_mps2_));
      const tf2::Quaternion tilt_correction = tf2::Quaternion::getIdentity().slerp(
        full_tilt_correction, accelerometer_tilt_correction_weight_);
      orientation_ = tilt_correction * orientation_;
      orientation_.normalize();
    }

    tf2::Quaternion midpoint_orientation = old_orientation.slerp(orientation_, 0.5);
    midpoint_orientation.normalize();
    // Gravity removal is never exact: accelerometer noise and tiny attitude errors leave a
    // residual which otherwise gets integrated twice into an unbounded position drift. Use the
    // same deadband advertised by the calibrated IMU output for estimator propagation as well.
    const tf2::Vector3 acceleration_world = apply_deadband(
      tf2::quatRotate(midpoint_orientation, acceleration_body) -
      tf2::Vector3(0.0, 0.0, gravity_mps2_),
      calibrated_accel_deadband_mps2_);
    position_ += velocity_ * dt + acceleration_world * (0.5 * dt * dt);
    velocity_ += acceleration_world * dt;
    angular_velocity_ = angular_velocity;
    last_imu_stamp_ = stamp;
    publish_calibrated_imu(
      stamp, angular_velocity, remove_gravity(acceleration_body));
    publish_odometry(stamp);
  }

  void start_full_calibration(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    reset_estimator_for_calibration(initialization_samples_);
    response->success = true;
    response->message = "full VIO calibration started; keep the drone stationary";
    RCLCPP_INFO(
      get_logger(),
      "Full VIO calibration started; resetting pose and collecting %d stationary samples",
      initialization_samples_);
  }

  void reset_estimator_for_calibration(int sample_count)
  {
    initialized_ = false;
    active_initialization_samples_ = sample_count;
    initialization_count_ = 0;
    gyro_sum_.setValue(0.0, 0.0, 0.0);
    acceleration_sum_.setValue(0.0, 0.0, 0.0);
    gyro_bias_.setValue(0.0, 0.0, 0.0);
    accel_scale_correction_ = 1.0;
    position_.setValue(0.0, 0.0, 0.0);
    velocity_.setValue(0.0, 0.0, 0.0);
    angular_velocity_.setValue(0.0, 0.0, 0.0);
    orientation_ = tf2::Quaternion::getIdentity();
    previous_image_position_.setValue(0.0, 0.0, 0.0);
    previous_image_orientation_ = tf2::Quaternion::getIdentity();
    previous_gray_.release();
    previous_points_.clear();
    accepted_visual_updates_ = 0;
    publish_calibration_status(false);
    publish_video_status(false);
    publish_visual_tracking(false);
  }

  void collect_initialization_sample(
    const tf2::Vector3 & gyro, const tf2::Vector3 & acceleration,
    const builtin_interfaces::msg::Time & stamp_message)
  {
    gyro_sum_ += gyro;
    acceleration_sum_ += acceleration;
    ++initialization_count_;
    if (initialization_count_ < active_initialization_samples_) {
      return;
    }

    gyro_bias_ = gyro_sum_ / static_cast<double>(initialization_count_);
    const tf2::Vector3 mean_acceleration =
      acceleration_sum_ / static_cast<double>(initialization_count_);
    const double measured_gravity = mean_acceleration.length();
    if (measured_gravity < 0.1) {
      initialization_count_ = 0;
      gyro_sum_.setValue(0.0, 0.0, 0.0);
      acceleration_sum_.setValue(0.0, 0.0, 0.0);
      RCLCPP_WARN(get_logger(), "Cannot calibrate: accelerometer magnitude is near zero");
      return;
    }
    accel_scale_correction_ = gravity_mps2_ / measured_gravity;
    if (accel_scale_correction_ < 0.5 || accel_scale_correction_ > 2.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Accelerometer scale correction %.3f is outside the expected 0.5-2.0 range; "
        "check the MPU identity and range-readback log",
        accel_scale_correction_);
    }
    const tf2::Vector3 scaled_mean_acceleration =
      mean_acceleration * accel_scale_correction_;
    orientation_ = shortest_arc(
      scaled_mean_acceleration, tf2::Vector3(0.0, 0.0, gravity_mps2_));
    orientation_.normalize();
    last_imu_stamp_ = rclcpp::Time(stamp_message);
    initialized_ = true;
    publish_calibration_status(true);
    RCLCPP_INFO(
      get_logger(),
      "VIO initialized after %d samples: gyro bias [%.5f %.5f %.5f], "
      "raw gravity %.5f m/s^2, accel scale %.5f",
      initialization_count_,
      gyro_bias_.x(), gyro_bias_.y(), gyro_bias_.z(),
      measured_gravity, accel_scale_correction_);
    publish_calibrated_imu(
      last_imu_stamp_, gyro - gyro_bias_,
      remove_gravity(acceleration * accel_scale_correction_));
    publish_odometry(last_imu_stamp_);
  }

  tf2::Vector3 remove_gravity(const tf2::Vector3 & acceleration_body) const
  {
    const tf2::Vector3 gravity_body = tf2::quatRotate(
      orientation_.inverse(), tf2::Vector3(0.0, 0.0, gravity_mps2_));
    return acceleration_body - gravity_body;
  }

  void publish_calibration_status(bool calibrated)
  {
    std_msgs::msg::Bool message;
    message.data = calibrated;
    calibration_status_publisher_->publish(message);
  }

  void publish_visual_tracking(bool tracking)
  {
    if (visual_tracking_status_published_ && tracking == visual_tracking_) {
      return;
    }
    visual_tracking_ = tracking;
    visual_tracking_status_published_ = true;
    std_msgs::msg::Bool message;
    message.data = tracking;
    visual_tracking_publisher_->publish(message);
  }

  void publish_video_status(bool working)
  {
    std_msgs::msg::Bool message;
    message.data = working;
    video_status_publisher_->publish(message);
  }

  void publish_calibrated_imu(
    const rclcpp::Time & stamp, const tf2::Vector3 & angular_velocity,
    const tf2::Vector3 & calibrated_acceleration)
  {
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = base_frame_;
    message.orientation = quaternion_message(orientation_);
    message.angular_velocity = vector_message(
      apply_deadband(angular_velocity, calibrated_gyro_deadband_rad_s_));
    message.linear_acceleration = vector_message(
      apply_deadband(calibrated_acceleration, calibrated_accel_deadband_mps2_));
    message.orientation_covariance[0] = orientation_variance_;
    message.orientation_covariance[4] = orientation_variance_;
    message.orientation_covariance[8] = orientation_variance_;
    message.angular_velocity_covariance[0] = angular_velocity_variance_;
    message.angular_velocity_covariance[4] = angular_velocity_variance_;
    message.angular_velocity_covariance[8] = angular_velocity_variance_;
    message.linear_acceleration_covariance[0] = linear_velocity_variance_;
    message.linear_acceleration_covariance[4] = linear_velocity_variance_;
    message.linear_acceleration_covariance[8] = linear_velocity_variance_;
    calibrated_imu_publisher_->publish(message);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    if (!initialized_) {
      return;
    }
    if (!has_intrinsics_) {
      publish_video_status(false);
      publish_visual_tracking(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "VIO has no camera intrinsics; waiting for %s or nonzero fx/fy parameters",
        camera_info_topic_.c_str());
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    if (!previous_gray_.empty()) {
      const double elapsed = (stamp - previous_image_stamp_).seconds();
      if (elapsed > 0.0 && elapsed < 1.0 / visual_processing_rate_hz_) {
        return;
      }
    }

    cv::Mat gray;
    try {
      gray = grayscale_image(message);
    } catch (const std::exception & error) {
      publish_video_status(false);
      publish_visual_tracking(false);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot convert VIO image: %s", error.what());
      return;
    }
    if (gray.empty()) {
      publish_video_status(false);
      publish_visual_tracking(false);
      return;
    }
    if (image_processing_scale_ < 0.999) {
      cv::Mat scaled;
      cv::resize(
        gray, scaled, cv::Size(), image_processing_scale_, image_processing_scale_,
        cv::INTER_AREA);
      gray = std::move(scaled);
    }
    // A successfully decoded frame with valid intrinsics proves that the camera side of VIO is
    // operational even when a stationary scene cannot produce a visual motion update.
    publish_video_status(true);

    if (previous_gray_.empty()) {
      reset_visual_reference(gray, stamp);
      publish_visual_tracking(false);
      return;
    }
    const double dt = (stamp - previous_image_stamp_).seconds();
    if (dt <= 0.0 || dt > max_image_gap_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Resetting visual tracker after invalid image interval %.3f s", dt);
      reset_visual_reference(gray, stamp);
      publish_visual_tracking(false);
      return;
    }
    if (previous_points_.size() < static_cast<std::size_t>(min_tracked_features_)) {
      reset_visual_reference(gray, stamp);
      publish_visual_tracking(false);
      return;
    }

    bool visual_update_accepted = false;
    try {
      std::vector<cv::Point2f> current_points;
      std::vector<unsigned char> status;
      std::vector<float> errors;
      cv::calcOpticalFlowPyrLK(
        previous_gray_, gray, previous_points_, current_points, status, errors,
        cv::Size(21, 21), 3);

      std::vector<cv::Point2f> valid_previous;
      std::vector<cv::Point2f> valid_current;
      valid_previous.reserve(current_points.size());
      valid_current.reserve(current_points.size());
      for (std::size_t index = 0; index < current_points.size(); ++index) {
        const auto & point = current_points[index];
        if (status[index] && point.x >= 1.0F && point.y >= 1.0F &&
          point.x < static_cast<float>(gray.cols - 1) &&
          point.y < static_cast<float>(gray.rows - 1))
        {
          valid_previous.push_back(previous_points_[index]);
          valid_current.push_back(point);
        }
      }

      if (valid_current.size() >= static_cast<std::size_t>(min_tracked_features_)) {
        visual_update_accepted = apply_visual_update(valid_previous, valid_current, dt);
      } else {
        RCLCPP_DEBUG(
          get_logger(), "Only %zu visual tracks survived; need %d",
          valid_current.size(), min_tracked_features_);
      }
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Visual tracking failed: %s", error.what());
    }
    publish_visual_tracking(visual_update_accepted);
    reset_visual_reference(gray, stamp);
  }

  cv::Mat grayscale_image(const sensor_msgs::msg::Image::ConstSharedPtr & message) const
  {
    return cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::MONO8)->image;
  }

  void reset_visual_reference(const cv::Mat & gray, const rclcpp::Time & stamp)
  {
    previous_gray_ = gray.clone();
    cv::goodFeaturesToTrack(
      previous_gray_, previous_points_, max_features_, feature_quality_, feature_min_distance_);
    previous_image_stamp_ = stamp;
    previous_image_position_ = position_;
    previous_image_orientation_ = orientation_;
  }

  bool apply_visual_update(
    const std::vector<cv::Point2f> & previous_points,
    const std::vector<cv::Point2f> & current_points,
    double dt)
  {
    const double scaled_fx = fx_ * image_processing_scale_;
    const double scaled_fy = fy_ * image_processing_scale_;
    const double scaled_cx = cx_ * image_processing_scale_;
    const double scaled_cy = cy_ * image_processing_scale_;
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      scaled_fx, 0.0, scaled_cx, 0.0, scaled_fy, scaled_cy, 0.0, 0.0, 1.0);
    const cv::Mat distortion = distortion_coefficients_.empty() ?
      cv::Mat() : cv::Mat(distortion_coefficients_).clone();
    std::vector<cv::Point2f> normalized_previous;
    std::vector<cv::Point2f> normalized_current;
    cv::undistortPoints(
      previous_points, normalized_previous, camera_matrix, distortion);
    cv::undistortPoints(
      current_points, normalized_current, camera_matrix, distortion);
    const cv::Mat normalized_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    const double normalized_ransac_threshold =
      (ransac_threshold_pixels_ * image_processing_scale_) /
      std::max(1.0, 0.5 * (scaled_fx + scaled_fy));
    cv::Mat inlier_mask;
    const cv::Mat essential = cv::findEssentialMat(
      normalized_previous, normalized_current, normalized_camera_matrix, cv::RANSAC,
      ransac_probability_, normalized_ransac_threshold, inlier_mask);
    if (essential.empty()) {
      return false;
    }

    cv::Mat relative_rotation;
    cv::Mat relative_translation;
    const int inliers = cv::recoverPose(
      essential, normalized_previous, normalized_current, normalized_camera_matrix,
      relative_rotation, relative_translation, inlier_mask);
    if (inliers < min_tracked_features_) {
      return false;
    }

    tf2::Matrix3x3 current_from_previous(
      relative_rotation.at<double>(0, 0), relative_rotation.at<double>(0, 1),
      relative_rotation.at<double>(0, 2), relative_rotation.at<double>(1, 0),
      relative_rotation.at<double>(1, 1), relative_rotation.at<double>(1, 2),
      relative_rotation.at<double>(2, 0), relative_rotation.at<double>(2, 1),
      relative_rotation.at<double>(2, 2));
    tf2::Quaternion relative_orientation;
    current_from_previous.getRotation(relative_orientation);
    relative_orientation.normalize();
    const double visual_angle = relative_orientation.getAngleShortestPath();
    if (!std::isfinite(visual_angle) || visual_angle > max_visual_rotation_rad_) {
      RCLCPP_DEBUG(get_logger(), "Rejected %.3f rad visual rotation", visual_angle);
      return false;
    }

    const tf2::Quaternion previous_camera_orientation =
      previous_image_orientation_ * camera_to_body_;
    tf2::Quaternion visual_camera_orientation =
      previous_camera_orientation * relative_orientation.inverse();
    tf2::Quaternion visual_body_orientation =
      visual_camera_orientation * camera_to_body_.inverse();
    visual_body_orientation.normalize();
    if (orientation_.dot(visual_body_orientation) < 0.0) {
      visual_body_orientation = tf2::Quaternion(
        -visual_body_orientation.x(), -visual_body_orientation.y(),
        -visual_body_orientation.z(), -visual_body_orientation.w());
    }
    orientation_ = orientation_.slerp(visual_body_orientation, visual_orientation_weight_);
    orientation_.normalize();

    const tf2::Vector3 translation_current(
      relative_translation.at<double>(0), relative_translation.at<double>(1),
      relative_translation.at<double>(2));
    const tf2::Vector3 direction_previous_camera =
      -(current_from_previous.transpose() * translation_current);
    const tf2::Vector3 direction_world =
      tf2::quatRotate(previous_camera_orientation, direction_previous_camera).normalized();
    const tf2::Vector3 inertial_displacement = position_ - previous_image_position_;
    const double scale = inertial_displacement.length();
    if (std::isfinite(scale) && scale > 1.0e-4 && scale <= maximum_visual_translation_m_) {
      const tf2::Vector3 visual_position = previous_image_position_ + direction_world * scale;
      const tf2::Vector3 fused_position =
        position_.lerp(visual_position, visual_translation_weight_);
      const tf2::Vector3 measured_velocity =
        (fused_position - previous_image_position_) / dt;
      position_ = fused_position;
      velocity_ = velocity_.lerp(measured_velocity, visual_velocity_weight_);
    }

    ++accepted_visual_updates_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Visual odometry active: %zu updates accepted; latest %d/%zu inliers, scale %.4f m",
      accepted_visual_updates_, inliers, current_points.size(), scale);
    return true;
  }

  void publish_odometry(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position = point_message(position_);
    odometry.pose.pose.orientation = quaternion_message(orientation_);
    odometry.twist.twist.linear = vector_message(
      tf2::quatRotate(orientation_.inverse(), velocity_));
    odometry.twist.twist.angular = vector_message(angular_velocity_);
    odometry.pose.covariance[0] = position_variance_;
    odometry.pose.covariance[7] = position_variance_;
    odometry.pose.covariance[14] = position_variance_;
    odometry.pose.covariance[21] = orientation_variance_;
    odometry.pose.covariance[28] = orientation_variance_;
    odometry.pose.covariance[35] = orientation_variance_;
    odometry.twist.covariance[0] = linear_velocity_variance_;
    odometry.twist.covariance[7] = linear_velocity_variance_;
    odometry.twist.covariance[14] = linear_velocity_variance_;
    odometry.twist.covariance[21] = angular_velocity_variance_;
    odometry.twist.covariance[28] = angular_velocity_variance_;
    odometry.twist.covariance[35] = angular_velocity_variance_;
    odometry_publisher_->publish(odometry);

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation = vector_message(position_);
      transform.transform.rotation = quaternion_message(orientation_);
      transform_broadcaster_->sendTransform(transform);
    }
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string imu_topic_;
  std::string calibrated_imu_topic_;
  std::string odom_topic_;
  std::string video_status_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  bool has_intrinsics_ = false;
  bool initialized_ = false;
  bool calibrate_on_startup_ = true;
  bool visual_tracking_ = false;
  bool visual_tracking_status_published_ = false;

  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  std::vector<double> distortion_coefficients_;
  int max_features_ = 300;
  int min_tracked_features_ = 35;
  double feature_quality_ = 0.01;
  double feature_min_distance_ = 12.0;
  double ransac_probability_ = 0.999;
  double ransac_threshold_pixels_ = 1.5;
  double max_visual_rotation_rad_ = 0.7;
  double visual_processing_rate_hz_ = 5.0;
  double image_processing_scale_ = 0.5;
  double visual_orientation_weight_ = 0.20;
  double visual_translation_weight_ = 0.25;
  double visual_velocity_weight_ = 0.10;
  double maximum_visual_translation_m_ = 2.0;

  double gravity_mps2_ = 9.80665;
  double calibrated_gyro_deadband_rad_s_ = 0.02;
  double calibrated_accel_deadband_mps2_ = 0.20;
  double accelerometer_tilt_correction_weight_ = 0.02;
  double accelerometer_gravity_tolerance_mps2_ = 1.5;
  double accel_scale_correction_ = 1.0;
  int initialization_samples_ = 20;
  int startup_initialization_samples_ = 1000;
  int active_initialization_samples_ = 20;
  int initialization_count_ = 0;
  std::size_t accepted_visual_updates_ = 0;
  double max_imu_gap_sec_ = 0.25;
  double max_image_gap_sec_ = 1.0;
  double position_variance_ = 0.05;
  double orientation_variance_ = 0.02;
  double linear_velocity_variance_ = 0.10;
  double angular_velocity_variance_ = 0.02;

  tf2::Quaternion camera_to_body_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion imu_to_body_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion orientation_{tf2::Quaternion::getIdentity()};
  tf2::Quaternion previous_image_orientation_{tf2::Quaternion::getIdentity()};
  tf2::Vector3 position_{0.0, 0.0, 0.0};
  tf2::Vector3 velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 angular_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_bias_{0.0, 0.0, 0.0};
  tf2::Vector3 gyro_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 acceleration_sum_{0.0, 0.0, 0.0};
  tf2::Vector3 previous_image_position_{0.0, 0.0, 0.0};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_image_stamp_{0, 0, RCL_ROS_TIME};
  cv::Mat previous_gray_;
  std::vector<cv::Point2f> previous_points_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr calibrated_imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_tracking_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr video_status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VioNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vio_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}

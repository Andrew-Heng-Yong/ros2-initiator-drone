#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace {

double fovFraction(double inner_fov_deg, double outer_fov_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double inner = std::tan((inner_fov_deg * kPi / 180.0) / 2.0);
  const double outer = std::tan((outer_fov_deg * kPi / 180.0) / 2.0);
  return outer > 0.0 ? std::clamp(inner / outer, 0.0, 1.0) : 1.0;
}

cv::Vec3b heatColor(double value) {
  const std::vector<cv::Vec3d> stops{
    {65.0, 28.0, 20.0},
    {183.0, 104.0, 37.0},
    {151.0, 194.0, 37.0},
    {36.0, 191.0, 251.0},
    {38.0, 38.0, 220.0},
  };
  const double position = std::clamp(value, 0.0, 0.999) * static_cast<double>(stops.size() - 1);
  const int start = static_cast<int>(std::floor(position));
  const int end = static_cast<int>(std::ceil(position));
  const double mix = position - static_cast<double>(start);
  const cv::Vec3d color = stops[start] + (stops[end] - stops[start]) * mix;
  return cv::Vec3b(
    static_cast<std::uint8_t>(std::clamp(color[0], 0.0, 255.0)),
    static_cast<std::uint8_t>(std::clamp(color[1], 0.0, 255.0)),
    static_cast<std::uint8_t>(std::clamp(color[2], 0.0, 255.0)));
}

}  // namespace

class ThermalOverlayNode final : public rclcpp::Node {
 public:
  ThermalOverlayNode() : Node("thermal_overlay_node") {
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
    thermal_topic_ = declare_parameter<std::string>("thermal_topic", "/thermal/image_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/thermal_overlay/image_raw");
    alpha_ = declare_parameter<double>("alpha", 0.45);
    camera_hfov_deg_ = declare_parameter<double>("camera_hfov_deg", 67.0);
    camera_vfov_deg_ = declare_parameter<double>("camera_vfov_deg", 53.6);
    thermal_hfov_deg_ = declare_parameter<double>("thermal_hfov_deg", 55.0);
    thermal_vfov_deg_ = declare_parameter<double>("thermal_vfov_deg", 35.0);

    alpha_ = std::clamp(alpha_, 0.0, 1.0);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "alpha") {
            const double next_alpha = parameter.as_double();
            if (next_alpha < 0.0 || next_alpha > 1.0) {
              result.successful = false;
              result.reason = "alpha must be between 0.0 and 1.0";
              return result;
            }
            alpha_ = next_alpha;
          }
        }
        return result;
      });

    thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr message) { thermalCallback(message); });

    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr message) { cameraCallback(message); });

    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "thermal overlay: camera=%s thermal=%s output=%s",
      camera_topic_.c_str(),
      thermal_topic_.c_str(),
      output_topic_.c_str());
  }

 private:
  void thermalCallback(const sensor_msgs::msg::Image::SharedPtr message) {
    if (message->encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&
        message->encoding != "32FC1")
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "thermal overlay expects 32FC1 thermal image, got %s", message->encoding.c_str());
      return;
    }

    const int rows = static_cast<int>(message->height);
    const int cols = static_cast<int>(message->width);
    if (rows <= 0 || cols <= 0 || message->data.size() < static_cast<std::size_t>(rows * cols * 4)) {
      return;
    }

    cv::Mat thermal(rows, cols, CV_32FC1);
    std::memcpy(thermal.data, message->data.data(), static_cast<std::size_t>(rows * cols * 4));
    latest_thermal_ = thermal.clone();
  }

  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr message) {
    if (latest_thermal_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "waiting for thermal image");
      return;
    }

    cv_bridge::CvImageConstPtr camera;
    try {
      camera = cv_bridge::toCvShare(message, message->encoding);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera conversion failed: %s", error.what());
      return;
    }

    cv::Mat rgb;
    if (message->encoding == sensor_msgs::image_encodings::RGB8) {
      rgb = camera->image.clone();
    } else if (message->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::cvtColor(camera->image, rgb, cv::COLOR_BGR2RGB);
    } else if (message->encoding == sensor_msgs::image_encodings::BGRA8) {
      cv::cvtColor(camera->image, rgb, cv::COLOR_BGRA2RGB);
    } else if (message->encoding == sensor_msgs::image_encodings::RGBA8) {
      cv::cvtColor(camera->image, rgb, cv::COLOR_RGBA2RGB);
    } else if (message->encoding == sensor_msgs::image_encodings::MONO8) {
      cv::cvtColor(camera->image, rgb, cv::COLOR_GRAY2RGB);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unsupported camera encoding for overlay: %s", message->encoding.c_str());
      return;
    }

    overlayThermal(rgb, latest_thermal_);

    cv_bridge::CvImage output;
    output.header = message->header;
    output.encoding = sensor_msgs::image_encodings::RGB8;
    output.image = rgb;
    overlay_pub_->publish(*output.toImageMsg());
  }

  void overlayThermal(cv::Mat & rgb, const cv::Mat & thermal) const {
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (int row = 0; row < thermal.rows; ++row) {
      for (int col = 0; col < thermal.cols; ++col) {
        const float value = thermal.at<float>(row, col);
        if (!std::isfinite(value)) continue;
        low = std::min(low, static_cast<double>(value));
        high = std::max(high, static_cast<double>(value));
      }
    }
    if (!std::isfinite(low) || !std::isfinite(high)) return;
    const double span = std::max(0.5, high - low);

    const int overlay_width = std::max(
      thermal.cols,
      static_cast<int>(std::round(rgb.cols * fovFraction(thermal_hfov_deg_, camera_hfov_deg_))));
    const int overlay_height = std::max(
      thermal.rows,
      static_cast<int>(std::round(rgb.rows * fovFraction(thermal_vfov_deg_, camera_vfov_deg_))));
    const int left = std::clamp((rgb.cols - overlay_width) / 2, 0, std::max(0, rgb.cols - 1));
    const int top = std::clamp((rgb.rows - overlay_height) / 2, 0, std::max(0, rgb.rows - 1));
    const int right = std::min(rgb.cols, left + overlay_width);
    const int bottom = std::min(rgb.rows, top + overlay_height);

    for (int y = top; y < bottom; ++y) {
      const int thermal_y = std::clamp(
        static_cast<int>((static_cast<double>(y - top) / std::max(1, bottom - top)) * thermal.rows),
        0,
        thermal.rows - 1);
      for (int x = left; x < right; ++x) {
        const int thermal_x = std::clamp(
          static_cast<int>((static_cast<double>(x - left) / std::max(1, right - left)) * thermal.cols),
          0,
          thermal.cols - 1);
        const float temperature = thermal.at<float>(thermal_y, thermal_x);
        if (!std::isfinite(temperature)) continue;

        const cv::Vec3b color = heatColor((static_cast<double>(temperature) - low) / span);
        cv::Vec3b & pixel = rgb.at<cv::Vec3b>(y, x);
        for (int channel = 0; channel < 3; ++channel) {
          pixel[channel] = static_cast<std::uint8_t>(
            std::round((1.0 - alpha_) * pixel[channel] + alpha_ * color[channel]));
        }
      }
    }
  }

  std::string camera_topic_;
  std::string thermal_topic_;
  std::string output_topic_;
  double alpha_{0.45};
  double camera_hfov_deg_{67.0};
  double camera_vfov_deg_{53.6};
  double thermal_hfov_deg_{55.0};
  double thermal_vfov_deg_{35.0};
  cv::Mat latest_thermal_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalOverlayNode>());
  rclcpp::shutdown();
  return 0;
}

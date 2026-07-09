#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

namespace
{

struct CropRegion
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
  int cluster_size = 0;
  int highlighted_count = 0;
};

int bytes_per_pixel(const sensor_msgs::msg::Image & image)
{
  if (image.width == 0 || image.step == 0) {
    return 0;
  }
  const auto by_step = static_cast<int>(image.step / image.width);
  if (by_step > 0) {
    return by_step;
  }

  const std::string encoding = image.encoding;
  if (
    encoding == "mono8" || encoding == "8UC1" || encoding == "bgr8" || encoding == "rgb8" ||
    encoding == "rgba8" || encoding == "bgra8")
  {
    if (encoding == "mono8" || encoding == "8UC1") {
      return 1;
    }
    return encoding == "rgba8" || encoding == "bgra8" ? 4 : 3;
  }
  if (encoding == "mono16" || encoding == "16UC1" || encoding == "16SC1") {
    return 2;
  }
  if (encoding == "32FC1") {
    return 4;
  }
  return 0;
}

bool read_thermal_value(const sensor_msgs::msg::Image & image, int x, int y, float & value)
{
  const std::string & encoding = image.encoding;
  const auto offset = static_cast<size_t>(y) * image.step;
  if (encoding == "32FC1") {
    const auto index = offset + static_cast<size_t>(x) * 4;
    if (index + 4 > image.data.size()) {
      return false;
    }
    std::memcpy(&value, image.data.data() + index, sizeof(float));
    return std::isfinite(value);
  }
  if (encoding == "16UC1" || encoding == "mono16") {
    const auto index = offset + static_cast<size_t>(x) * 2;
    if (index + 2 > image.data.size()) {
      return false;
    }
    uint16_t raw = 0;
    std::memcpy(&raw, image.data.data() + index, sizeof(uint16_t));
    value = static_cast<float>(raw);
    return true;
  }
  if (encoding == "16SC1") {
    const auto index = offset + static_cast<size_t>(x) * 2;
    if (index + 2 > image.data.size()) {
      return false;
    }
    int16_t raw = 0;
    std::memcpy(&raw, image.data.data() + index, sizeof(int16_t));
    value = static_cast<float>(raw);
    return true;
  }
  if (encoding == "8UC1" || encoding == "mono8") {
    const auto index = offset + static_cast<size_t>(x);
    if (index >= image.data.size()) {
      return false;
    }
    value = static_cast<float>(image.data[index]);
    return true;
  }
  return false;
}

sensor_msgs::msg::Image crop_image(const sensor_msgs::msg::Image & image, const CropRegion & roi)
{
  sensor_msgs::msg::Image output;
  output.header = image.header;
  output.height = static_cast<uint32_t>(roi.height);
  output.width = static_cast<uint32_t>(roi.width);
  output.encoding = image.encoding;
  output.is_bigendian = image.is_bigendian;

  const int bpp = bytes_per_pixel(image);
  if (bpp <= 0 || roi.width <= 0 || roi.height <= 0) {
    return output;
  }

  output.step = static_cast<uint32_t>(roi.width * bpp);
  output.data.resize(static_cast<size_t>(output.step) * output.height);
  for (int y = 0; y < roi.height; ++y) {
    const auto source = static_cast<size_t>(roi.y + y) * image.step +
      static_cast<size_t>(roi.x) * bpp;
    const auto target = static_cast<size_t>(y) * output.step;
    const auto bytes = static_cast<size_t>(roi.width) * bpp;
    if (source + bytes <= image.data.size() && target + bytes <= output.data.size()) {
      std::copy_n(image.data.begin() + static_cast<long>(source), bytes, output.data.begin() + static_cast<long>(target));
    }
  }
  return output;
}

sensor_msgs::msg::CameraInfo crop_camera_info(
  const sensor_msgs::msg::CameraInfo & info,
  const CropRegion & roi)
{
  auto output = info;
  output.width = static_cast<uint32_t>(roi.width);
  output.height = static_cast<uint32_t>(roi.height);
  output.k[2] -= roi.x;
  output.k[5] -= roi.y;
  output.p[2] -= roi.x;
  output.p[6] -= roi.y;
  output.roi.x_offset = static_cast<uint32_t>(roi.x);
  output.roi.y_offset = static_cast<uint32_t>(roi.y);
  output.roi.width = static_cast<uint32_t>(roi.width);
  output.roi.height = static_cast<uint32_t>(roi.height);
  output.roi.do_rectify = false;
  return output;
}

}  // namespace

class ThermalCropperNode : public rclcpp::Node
{
public:
  ThermalCropperNode()
  : Node("thermal_cropper_node")
  {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    depth_camera_info_topic_ = declare_parameter<std::string>(
      "depth_camera_info_topic", "/camera/depth/camera_info");
    thermal_topic_ = declare_parameter<std::string>("thermal_topic", "/thermal/image_raw");
    output_depth_topic_ = declare_parameter<std::string>(
      "output_depth_topic", "/camera/depth/cropped/image_raw");
    output_thermal_topic_ = declare_parameter<std::string>(
      "output_thermal_topic", "/thermal/cropped/image_raw");
    output_camera_info_topic_ = declare_parameter<std::string>(
      "output_camera_info_topic", "/camera/depth/cropped/camera_info");
    output_roi_topic_ = declare_parameter<std::string>(
      "output_roi_topic", "/thermal/crop_region");

    enabled_ = declare_parameter<bool>("enabled", true);
    crop_unit_thermal_pixels_ = declare_parameter<int>("crop_unit_thermal_pixels", 1);
    min_region_size_ = declare_parameter<int>("min_region_size", 4);
    inflation_radius_thermal_pixels_ = declare_parameter<int>(
      "inflation_radius_thermal_pixels", 1);
    highlight_min_temp_ = declare_parameter<double>("highlight_min_temp", 30.0);
    highlight_max_temp_ = declare_parameter<double>("highlight_max_temp", 120.0);
    highlight_min_delta_from_frame_low_ = declare_parameter<double>(
      "highlight_min_delta_from_frame_low", 3.0);
    highlight_max_delta_from_frame_high_ = declare_parameter<double>(
      "highlight_max_delta_from_frame_high", 1000.0);

    depth_hfov_deg_ = declare_parameter<double>("depth_fov_horizontal", 67.0);
    depth_vfov_deg_ = declare_parameter<double>("depth_fov_vertical", 53.6);
    thermal_hfov_deg_ = declare_parameter<double>("thermal_fov_horizontal", 55.0);
    thermal_vfov_deg_ = declare_parameter<double>("thermal_fov_vertical", 35.0);
    thermal_offset_x_ = declare_parameter<double>("thermal_offset_x", 10.0);
    thermal_offset_y_ = declare_parameter<double>("thermal_offset_y", 0.0);
    thermal_scale_ = declare_parameter<double>("thermal_scale", 0.8);
    thermal_stretch_x_ = declare_parameter<double>("thermal_stretch_x", 0.8);
    thermal_stretch_y_ = declare_parameter<double>("thermal_stretch_y", 1.0);
    flip_thermal_x_ = declare_parameter<bool>("flip_thermal_x", true);
    passthrough_when_no_region_ = declare_parameter<bool>("passthrough_when_no_region", true);

    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(output_depth_topic_, 10);
    thermal_pub_ = create_publisher<sensor_msgs::msg::Image>(output_thermal_topic_, 10);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(output_camera_info_topic_, 10);
    roi_pub_ = create_publisher<sensor_msgs::msg::RegionOfInterest>(output_roi_topic_, 10);

    thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_thermal(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_depth(*msg);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_camera_info_topic_, 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        latest_camera_info_ = *msg;
        have_camera_info_ = true;
      });

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters(parameters);
      });
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "enabled") {
        enabled_ = parameter.as_bool();
      } else if (name == "crop_unit_thermal_pixels") {
        crop_unit_thermal_pixels_ = std::max(1, parameter.as_int());
      } else if (name == "min_region_size") {
        min_region_size_ = std::max(1, parameter.as_int());
      } else if (name == "inflation_radius_thermal_pixels") {
        inflation_radius_thermal_pixels_ = std::max(0, parameter.as_int());
      } else if (name == "highlight_min_temp") {
        highlight_min_temp_ = parameter.as_double();
      } else if (name == "highlight_max_temp") {
        highlight_max_temp_ = parameter.as_double();
      } else if (name == "highlight_min_delta_from_frame_low") {
        highlight_min_delta_from_frame_low_ = std::max(0.0, parameter.as_double());
      } else if (name == "highlight_max_delta_from_frame_high") {
        highlight_max_delta_from_frame_high_ = std::max(0.0, parameter.as_double());
      } else if (name == "passthrough_when_no_region") {
        passthrough_when_no_region_ = parameter.as_bool();
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void handle_thermal(const sensor_msgs::msg::Image & image)
  {
    if (!enabled_) {
      have_crop_ = false;
      if (passthrough_when_no_region_) {
        thermal_pub_->publish(image);
      }
      return;
    }
    latest_thermal_width_ = static_cast<int>(image.width);
    latest_thermal_height_ = static_cast<int>(image.height);
    latest_crop_ = detect_crop(image);
    have_crop_ = latest_crop_.width > 0 && latest_crop_.height > 0;
    if (have_crop_) {
      thermal_pub_->publish(crop_image(image, latest_crop_));
    } else if (passthrough_when_no_region_) {
      thermal_pub_->publish(image);
    }
  }

  void handle_depth(const sensor_msgs::msg::Image & image)
  {
    if (!enabled_) {
      if (passthrough_when_no_region_) {
        depth_pub_->publish(image);
        if (have_camera_info_) {
          auto info = latest_camera_info_;
          info.header = image.header;
          camera_info_pub_->publish(info);
        }
      }
      return;
    }
    CropRegion roi;
    if (have_crop_) {
      roi = thermal_to_depth_roi(latest_crop_, static_cast<int>(image.width), static_cast<int>(image.height));
    } else if (passthrough_when_no_region_) {
      depth_pub_->publish(image);
      if (have_camera_info_) {
        auto info = latest_camera_info_;
        info.header = image.header;
        camera_info_pub_->publish(info);
      }
      return;
    } else {
      return;
    }

    roi.x = std::clamp(roi.x, 0, static_cast<int>(image.width) - 1);
    roi.y = std::clamp(roi.y, 0, static_cast<int>(image.height) - 1);
    roi.width = std::clamp(roi.width, 1, static_cast<int>(image.width) - roi.x);
    roi.height = std::clamp(roi.height, 1, static_cast<int>(image.height) - roi.y);

    depth_pub_->publish(crop_image(image, roi));
    if (have_camera_info_) {
      auto info = crop_camera_info(latest_camera_info_, roi);
      info.header = image.header;
      camera_info_pub_->publish(info);
    }
    sensor_msgs::msg::RegionOfInterest msg;
    msg.x_offset = static_cast<uint32_t>(roi.x);
    msg.y_offset = static_cast<uint32_t>(roi.y);
    msg.width = static_cast<uint32_t>(roi.width);
    msg.height = static_cast<uint32_t>(roi.height);
    msg.do_rectify = false;
    roi_pub_->publish(msg);
  }

  CropRegion detect_crop(const sensor_msgs::msg::Image & image)
  {
    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    if (width <= 0 || height <= 0) {
      return {};
    }

    std::vector<float> values(static_cast<size_t>(width * height), 0.0F);
    float low = std::numeric_limits<float>::infinity();
    float high = -std::numeric_limits<float>::infinity();
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        float value = 0.0F;
        if (!read_thermal_value(image, x, y, value)) {
          values[static_cast<size_t>(y * width + x)] = std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        values[static_cast<size_t>(y * width + x)] = value;
        low = std::min(low, value);
        high = std::max(high, value);
      }
    }
    if (!std::isfinite(low) || !std::isfinite(high)) {
      return {};
    }

    const int unit = std::clamp(crop_unit_thermal_pixels_, 1, std::max(width, height));
    const int columns = static_cast<int>(std::ceil(static_cast<double>(width) / unit));
    const int rows = static_cast<int>(std::ceil(static_cast<double>(height) / unit));
    std::vector<uint8_t> mask(static_cast<size_t>(columns * rows), 0);
    int highlighted_count = 0;

    for (int cell_y = 0; cell_y < rows; ++cell_y) {
      for (int cell_x = 0; cell_x < columns; ++cell_x) {
        bool highlighted = false;
        for (int y = cell_y * unit; y < std::min(height, (cell_y + 1) * unit) && !highlighted; ++y) {
          for (int x = cell_x * unit; x < std::min(width, (cell_x + 1) * unit); ++x) {
            const auto value = values[static_cast<size_t>(y * width + x)];
            if (is_highlighted(value, low, high)) {
              highlighted = true;
              break;
            }
          }
        }
        if (highlighted) {
          mask[static_cast<size_t>(cell_y * columns + cell_x)] = 1;
          ++highlighted_count;
        }
      }
    }

    std::vector<uint8_t> visited(mask.size(), 0);
    CropRegion best;
    for (int index = 0; index < static_cast<int>(mask.size()); ++index) {
      if (!mask[static_cast<size_t>(index)] || visited[static_cast<size_t>(index)]) {
        continue;
      }
      std::queue<int> queue;
      queue.push(index);
      visited[static_cast<size_t>(index)] = 1;
      int count = 0;
      int min_x = columns;
      int min_y = rows;
      int max_x = 0;
      int max_y = 0;

      while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        const int x = current % columns;
        const int y = current / columns;
        ++count;
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= columns || ny < 0 || ny >= rows) {
              continue;
            }
            const int next = ny * columns + nx;
            if (!mask[static_cast<size_t>(next)] || visited[static_cast<size_t>(next)]) {
              continue;
            }
            visited[static_cast<size_t>(next)] = 1;
            queue.push(next);
          }
        }
      }

      if (count > best.cluster_size) {
        const int inflate = std::max(0, inflation_radius_thermal_pixels_);
        const int left = std::max(0, min_x * unit - inflate);
        const int top = std::max(0, min_y * unit - inflate);
        const int right = std::min(width, (max_x + 1) * unit + inflate);
        const int bottom = std::min(height, (max_y + 1) * unit + inflate);
        best = {left, top, right - left, bottom - top, count, highlighted_count};
      }
    }

    if (best.cluster_size < min_region_size_) {
      return {};
    }
    return best;
  }

  bool is_highlighted(float value, float low, float high) const
  {
    if (!std::isfinite(value)) {
      return false;
    }
    if (value < highlight_min_temp_ || value > highlight_max_temp_) {
      return false;
    }
    if (value < low + highlight_min_delta_from_frame_low_) {
      return false;
    }
    if (value < high - highlight_max_delta_from_frame_high_) {
      return false;
    }
    return true;
  }

  CropRegion thermal_to_depth_roi(const CropRegion & thermal, int depth_width, int depth_height) const
  {
    const double depth_fraction_x = fov_fraction(thermal_hfov_deg_, depth_hfov_deg_) *
      thermal_scale_ * thermal_stretch_x_;
    const double depth_fraction_y = fov_fraction(thermal_vfov_deg_, depth_vfov_deg_) *
      thermal_scale_ * thermal_stretch_y_;
    const int thermal_window_width = std::max(
      latest_thermal_width_, static_cast<int>(std::round(depth_width * depth_fraction_x)));
    const int thermal_window_height = std::max(
      latest_thermal_height_, static_cast<int>(std::round(depth_height * depth_fraction_y)));
    const int thermal_window_left = static_cast<int>(
      std::round((depth_width - thermal_window_width) / 2.0 + thermal_offset_x_));
    const int thermal_window_top = static_cast<int>(
      std::round((depth_height - thermal_window_height) / 2.0 + thermal_offset_y_));

    const int display_x = flip_thermal_x_ ?
      latest_thermal_width_ - thermal.x - thermal.width :
      thermal.x;
    return {
      static_cast<int>(std::round(
        thermal_window_left + display_x * thermal_window_width /
        static_cast<double>(std::max(1, latest_thermal_width_)))),
      static_cast<int>(std::round(
        thermal_window_top + thermal.y * thermal_window_height /
        static_cast<double>(std::max(1, latest_thermal_height_)))),
      std::max(1, static_cast<int>(std::round(
        thermal.width * thermal_window_width /
        static_cast<double>(std::max(1, latest_thermal_width_))))),
      std::max(1, static_cast<int>(std::round(
        thermal.height * thermal_window_height /
        static_cast<double>(std::max(1, latest_thermal_height_))))),
      thermal.cluster_size,
      thermal.highlighted_count};
  }

  double fov_fraction(double inner_degrees, double outer_degrees) const
  {
    constexpr double k_pi = 3.14159265358979323846;
    const double inner = std::tan((inner_degrees * k_pi / 180.0) / 2.0);
    const double outer = std::tan((outer_degrees * k_pi / 180.0) / 2.0);
    return outer > 0.0 ? std::clamp(inner / outer, 0.0, 1.0) : 1.0;
  }

  std::string depth_topic_;
  std::string depth_camera_info_topic_;
  std::string thermal_topic_;
  std::string output_depth_topic_;
  std::string output_thermal_topic_;
  std::string output_camera_info_topic_;
  std::string output_roi_topic_;

  int crop_unit_thermal_pixels_ = 1;
  int min_region_size_ = 4;
  int inflation_radius_thermal_pixels_ = 1;
  double highlight_min_temp_ = 30.0;
  double highlight_max_temp_ = 120.0;
  double highlight_min_delta_from_frame_low_ = 3.0;
  double highlight_max_delta_from_frame_high_ = 1000.0;
  double depth_hfov_deg_ = 67.0;
  double depth_vfov_deg_ = 53.6;
  double thermal_hfov_deg_ = 55.0;
  double thermal_vfov_deg_ = 35.0;
  double thermal_offset_x_ = 10.0;
  double thermal_offset_y_ = 0.0;
  double thermal_scale_ = 0.8;
  double thermal_stretch_x_ = 0.8;
  double thermal_stretch_y_ = 1.0;
  bool flip_thermal_x_ = true;
  bool passthrough_when_no_region_ = true;
  bool enabled_ = true;

  CropRegion latest_crop_;
  bool have_crop_ = false;
  int latest_thermal_width_ = 32;
  int latest_thermal_height_ = 24;
  sensor_msgs::msg::CameraInfo latest_camera_info_;
  bool have_camera_info_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_pub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalCropperNode>());
  rclcpp::shutdown();
  return 0;
}

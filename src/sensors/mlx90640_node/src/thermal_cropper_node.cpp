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

sensor_msgs::msg::Image black_image_like(const sensor_msgs::msg::Image & image)
{
  auto output = image;
  std::fill(output.data.begin(), output.data.end(), 0);
  return output;
}

sensor_msgs::msg::Image mask_image_by_pixel_mask(
  const sensor_msgs::msg::Image & image,
  const std::vector<uint8_t> & mask)
{
  const int bpp = bytes_per_pixel(image);
  const auto pixel_count = static_cast<size_t>(image.width) * static_cast<size_t>(image.height);
  if (bpp <= 0 || mask.size() != pixel_count) {
    return black_image_like(image);
  }

  auto output = image;
  for (uint32_t y = 0; y < image.height; ++y) {
    for (uint32_t x = 0; x < image.width; ++x) {
      if (mask[static_cast<size_t>(y) * image.width + x]) {
        continue;
      }
      const auto offset = static_cast<size_t>(y) * image.step + static_cast<size_t>(x) * bpp;
      if (offset + static_cast<size_t>(bpp) <= output.data.size()) {
        std::fill_n(output.data.begin() + static_cast<long>(offset), bpp, 0);
      }
    }
  }
  return output;
}

int decimated_length(int length, int decimation)
{
  return (length + decimation - 1) / decimation;
}

sensor_msgs::msg::Image crop_and_decimate_image(
  const sensor_msgs::msg::Image & image,
  const CropRegion & roi,
  int decimation,
  const std::vector<int> * selected_pixels = nullptr)
{
  const int bpp = bytes_per_pixel(image);
  const int source_width = static_cast<int>(image.width);
  const int source_height = static_cast<int>(image.height);
  decimation = std::max(1, decimation);
  if (
    bpp <= 0 || roi.x < 0 || roi.y < 0 || roi.width <= 0 || roi.height <= 0 ||
    roi.x + roi.width > source_width || roi.y + roi.height > source_height)
  {
    return black_image_like(image);
  }

  auto output = image;
  output.width = static_cast<uint32_t>(decimated_length(roi.width, decimation));
  output.height = static_cast<uint32_t>(decimated_length(roi.height, decimation));
  output.step = output.width * static_cast<uint32_t>(bpp);
  output.data.assign(static_cast<size_t>(output.step) * output.height, 0);

  const auto copy_pixel = [&](int source_x, int source_y) {
    const int relative_x = source_x - roi.x;
    const int relative_y = source_y - roi.y;
    if (relative_x % decimation != 0 || relative_y % decimation != 0) {
      return;
    }
    const int output_x = relative_x / decimation;
    const int output_y = relative_y / decimation;
    const auto source_offset =
      static_cast<size_t>(source_y) * image.step + static_cast<size_t>(source_x) * bpp;
    const auto output_offset =
      static_cast<size_t>(output_y) * output.step + static_cast<size_t>(output_x) * bpp;
    if (
      source_offset + static_cast<size_t>(bpp) <= image.data.size() &&
      output_offset + static_cast<size_t>(bpp) <= output.data.size())
    {
      std::copy_n(
        image.data.begin() + static_cast<long>(source_offset), bpp,
        output.data.begin() + static_cast<long>(output_offset));
    }
  };

  if (selected_pixels != nullptr) {
    for (const int source_pixel : *selected_pixels) {
      copy_pixel(source_pixel % source_width, source_pixel / source_width);
    }
  } else {
    for (int output_y = 0; output_y < static_cast<int>(output.height); ++output_y) {
      const int source_y = roi.y + output_y * decimation;
      for (int output_x = 0; output_x < static_cast<int>(output.width); ++output_x) {
        copy_pixel(roi.x + output_x * decimation, source_y);
      }
    }
  }
  return output;
}

sensor_msgs::msg::CameraInfo crop_and_decimate_camera_info(
  const sensor_msgs::msg::CameraInfo & info,
  const CropRegion & roi,
  int decimation)
{
  decimation = std::max(1, decimation);
  const double scale = 1.0 / static_cast<double>(decimation);
  auto output = info;
  output.width = static_cast<uint32_t>(decimated_length(roi.width, decimation));
  output.height = static_cast<uint32_t>(decimated_length(roi.height, decimation));

  output.k[0] *= scale;
  output.k[2] = (output.k[2] - roi.x) * scale;
  output.k[4] *= scale;
  output.k[5] = (output.k[5] - roi.y) * scale;

  output.p[0] *= scale;
  output.p[2] = (output.p[2] - roi.x) * scale;
  output.p[3] *= scale;
  output.p[5] *= scale;
  output.p[6] = (output.p[6] - roi.y) * scale;
  output.p[7] *= scale;

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

    depth_hfov_deg_ = declare_parameter<double>("depth_fov_horizontal", 79.0);
    depth_vfov_deg_ = declare_parameter<double>("depth_fov_vertical", 62.0);
    thermal_hfov_deg_ = declare_parameter<double>("thermal_fov_horizontal", 90.0);
    thermal_vfov_deg_ = declare_parameter<double>("thermal_fov_vertical", 68.0);
    thermal_offset_x_ = declare_parameter<double>("thermal_offset_x", 0.0);
    thermal_offset_y_ = declare_parameter<double>("thermal_offset_y", 0.0);
    thermal_scale_ = declare_parameter<double>("thermal_scale", 1.0);
    thermal_barrel_distortion_ = declare_parameter<double>("thermal_barrel_distortion", 0.0);
    thermal_stretch_x_ = declare_parameter<double>("thermal_stretch_x", 0.8);
    thermal_stretch_y_ = declare_parameter<double>("thermal_stretch_y", 0.9);
    flip_thermal_x_ = declare_parameter<bool>("flip_thermal_x", true);
    flip_thermal_y_ = declare_parameter<bool>("flip_thermal_y", false);
    passthrough_when_no_region_ = declare_parameter<bool>("passthrough_when_no_region", true);
    output_decimation_ = static_cast<int>(std::clamp<int64_t>(
      declare_parameter<int64_t>("output_decimation", 2), 1, 8));

    // These streams are live visualizations: once a newer sample exists, an
    // older one only adds latency. Keep a single reliable output so rosbridge
    // cannot build a ROS-side history of obsolete depth frames.
    const auto latest_output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_depth_topic_, latest_output_qos);
    thermal_pub_ = create_publisher<sensor_msgs::msg::Image>(
      output_thermal_topic_, latest_output_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      output_camera_info_topic_, latest_output_qos);
    roi_pub_ = create_publisher<sensor_msgs::msg::RegionOfInterest>(
      output_roi_topic_, latest_output_qos);

    const auto latest_sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    thermal_sub_ = create_subscription<sensor_msgs::msg::Image>(
      thermal_topic_, latest_sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_thermal(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, latest_sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_depth(*msg);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        latest_camera_info_ = *msg;
        have_camera_info_ = true;
      });

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters(parameters);
      });

    RCLCPP_INFO(
      get_logger(),
      "Thermal cropper enabled=%s passthrough_when_no_region=%s min_region_size=%d "
      "barrel_distortion=%.3f output_decimation=%d",
      enabled_ ? "true" : "false",
      passthrough_when_no_region_ ? "true" : "false",
      min_region_size_,
      thermal_barrel_distortion_,
      output_decimation_);
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    bool geometry_changed = false;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "enabled") {
        enabled_ = parameter.as_bool();
      } else if (name == "crop_unit_thermal_pixels") {
        crop_unit_thermal_pixels_ = static_cast<int>(std::max<int64_t>(1, parameter.as_int()));
      } else if (name == "min_region_size") {
        min_region_size_ = static_cast<int>(std::max<int64_t>(1, parameter.as_int()));
      } else if (name == "inflation_radius_thermal_pixels") {
        inflation_radius_thermal_pixels_ = static_cast<int>(std::max<int64_t>(0, parameter.as_int()));
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
      } else if (name == "output_decimation") {
        output_decimation_ = static_cast<int>(std::clamp<int64_t>(parameter.as_int(), 1, 8));
      } else if (name == "depth_fov_horizontal") {
        depth_hfov_deg_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "depth_fov_vertical") {
        depth_vfov_deg_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "thermal_fov_horizontal") {
        thermal_hfov_deg_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "thermal_fov_vertical") {
        thermal_vfov_deg_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "thermal_offset_x") {
        thermal_offset_x_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "thermal_offset_y") {
        thermal_offset_y_ = parameter.as_double();
        geometry_changed = true;
      } else if (name == "thermal_scale") {
        thermal_scale_ = std::max(0.1, parameter.as_double());
        geometry_changed = true;
      } else if (name == "thermal_barrel_distortion") {
        thermal_barrel_distortion_ = std::clamp(parameter.as_double(), -1.0, 1.0);
        geometry_changed = true;
      } else if (name == "thermal_stretch_x") {
        thermal_stretch_x_ = std::max(0.1, parameter.as_double());
        geometry_changed = true;
      } else if (name == "thermal_stretch_y") {
        thermal_stretch_y_ = std::max(0.1, parameter.as_double());
        geometry_changed = true;
      } else if (name == "flip_thermal_x") {
        flip_thermal_x_ = parameter.as_bool();
        geometry_changed = true;
      } else if (name == "flip_thermal_y") {
        flip_thermal_y_ = parameter.as_bool();
        geometry_changed = true;
      }
    }
    if (geometry_changed) {
      thermal_to_depth_pixels_.clear();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void handle_thermal(const sensor_msgs::msg::Image & image)
  {
    if (!enabled_) {
      have_crop_ = false;
      latest_thermal_mask_.clear();
      if (passthrough_when_no_region_) {
        thermal_pub_->publish(image);
      }
      return;
    }
    latest_thermal_width_ = static_cast<int>(image.width);
    latest_thermal_height_ = static_cast<int>(image.height);
    const auto previous_mask = latest_thermal_mask_;
    const auto detected_crop = detect_crop(image);
    const bool detected = detected_crop.width > 0 && detected_crop.height > 0;
    if (detected) {
      latest_crop_ = detected_crop;
      have_crop_ = true;
      thermal_pub_->publish(mask_image_by_pixel_mask(image, latest_thermal_mask_));
    } else if (passthrough_when_no_region_) {
      have_crop_ = false;
      thermal_pub_->publish(image);
    } else if (have_crop_) {
      latest_thermal_mask_ = previous_mask;
      thermal_pub_->publish(mask_image_by_pixel_mask(image, latest_thermal_mask_));
    }
  }

  void handle_depth(const sensor_msgs::msg::Image & image)
  {
    if (!enabled_) {
      if (passthrough_when_no_region_) {
        publish_uncropped_depth(image);
      }
      return;
    }
    CropRegion roi;
    if (have_crop_) {
      auto output = crop_depth_by_thermal_mask(image, roi);
      if (roi.width <= 0 || roi.height <= 0) {
        if (passthrough_when_no_region_) {
          publish_uncropped_depth(image);
        }
        return;
      }
      log_depth_reduction(image, output, true);
      depth_pub_->publish(std::move(output));
    } else {
      if (passthrough_when_no_region_) {
        publish_uncropped_depth(image);
      }
      return;
    }

    if (have_camera_info_) {
      auto info = crop_and_decimate_camera_info(latest_camera_info_, roi, output_decimation_);
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

  void publish_uncropped_depth(const sensor_msgs::msg::Image & image)
  {
    const CropRegion full_frame{
      0, 0, static_cast<int>(image.width), static_cast<int>(image.height), 0, 0};
    auto output = crop_and_decimate_image(image, full_frame, output_decimation_);
    log_depth_reduction(image, output, false);
    depth_pub_->publish(std::move(output));
    if (have_camera_info_) {
      auto info = crop_and_decimate_camera_info(
        latest_camera_info_, full_frame, output_decimation_);
      info.header = image.header;
      camera_info_pub_->publish(info);
    }
  }

  void log_depth_reduction(
    const sensor_msgs::msg::Image & input,
    const sensor_msgs::msg::Image & output,
    bool cropped)
  {
    const double reduction = output.data.empty() ? 0.0 :
      static_cast<double>(input.data.size()) / static_cast<double>(output.data.size());
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Depth output %ux%u (%zu bytes) -> %ux%u (%zu bytes), %.1fx smaller, %s",
      input.width, input.height, input.data.size(), output.width, output.height,
      output.data.size(), reduction, cropped ? "thermal ROI" : "full-FOV fallback");
  }

  CropRegion detect_crop(const sensor_msgs::msg::Image & image)
  {
    latest_thermal_mask_.clear();
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
    std::vector<int> best_cells;
    int best_cluster_size = 0;
    for (int index = 0; index < static_cast<int>(mask.size()); ++index) {
      if (!mask[static_cast<size_t>(index)] || visited[static_cast<size_t>(index)]) {
        continue;
      }
      std::queue<int> queue;
      std::vector<int> cells;
      queue.push(index);
      visited[static_cast<size_t>(index)] = 1;

      while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        cells.push_back(current);
        const int x = current % columns;
        const int y = current / columns;

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

      if (static_cast<int>(cells.size()) > best_cluster_size) {
        best_cluster_size = static_cast<int>(cells.size());
        best_cells = std::move(cells);
      }
    }

    if (best_cluster_size < min_region_size_) {
      return {};
    }

    latest_thermal_mask_.assign(static_cast<size_t>(width * height), 0);
    const int inflate = std::max(0, inflation_radius_thermal_pixels_);
    const int inflate_squared = inflate * inflate;
    for (const int cell : best_cells) {
      const int cell_x = cell % columns;
      const int cell_y = cell / columns;
      const int source_left = cell_x * unit;
      const int source_top = cell_y * unit;
      const int source_right = std::min(width, (cell_x + 1) * unit);
      const int source_bottom = std::min(height, (cell_y + 1) * unit);

      for (int source_y = source_top; source_y < source_bottom; ++source_y) {
        for (int source_x = source_left; source_x < source_right; ++source_x) {
          for (int dy = -inflate; dy <= inflate; ++dy) {
            for (int dx = -inflate; dx <= inflate; ++dx) {
              if (dx * dx + dy * dy > inflate_squared) {
                continue;
              }
              const int x = source_x + dx;
              const int y = source_y + dy;
              if (x < 0 || x >= width || y < 0 || y >= height) {
                continue;
              }
              latest_thermal_mask_[static_cast<size_t>(y * width + x)] = 1;
            }
          }
        }
      }
    }

    int left = width;
    int top = height;
    int right = -1;
    int bottom = -1;
    int selected_pixels = 0;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!latest_thermal_mask_[static_cast<size_t>(y * width + x)]) {
          continue;
        }
        ++selected_pixels;
        left = std::min(left, x);
        top = std::min(top, y);
        right = std::max(right, x);
        bottom = std::max(bottom, y);
      }
    }
    if (selected_pixels == 0) {
      return {};
    }
    return {
      left,
      top,
      right - left + 1,
      bottom - top + 1,
      best_cluster_size,
      highlighted_count};
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

  sensor_msgs::msg::Image crop_depth_by_thermal_mask(
    const sensor_msgs::msg::Image & image,
    CropRegion & roi)
  {
    roi = {};
    const int bpp = bytes_per_pixel(image);
    const int depth_width = static_cast<int>(image.width);
    const int depth_height = static_cast<int>(image.height);
    const int thermal_width = latest_thermal_width_;
    const int thermal_height = latest_thermal_height_;
    if (
      bpp <= 0 || depth_width <= 0 || depth_height <= 0 || thermal_width <= 0 ||
      thermal_height <= 0 || latest_thermal_mask_.size() !=
      static_cast<size_t>(thermal_width * thermal_height))
    {
      return black_image_like(image);
    }
    ensure_thermal_to_depth_lookup(depth_width, depth_height, thermal_width, thermal_height);

    std::vector<int> selected_depth_pixels;
    int selected_left = depth_width;
    int selected_top = depth_height;
    int selected_right = -1;
    int selected_bottom = -1;
    for (int thermal_pixel = 0; thermal_pixel < thermal_width * thermal_height; ++thermal_pixel) {
      if (!latest_thermal_mask_[static_cast<size_t>(thermal_pixel)]) {
        continue;
      }
      for (const int depth_pixel : thermal_to_depth_pixels_[static_cast<size_t>(thermal_pixel)]) {
        const int y = depth_pixel / depth_width;
        const int x = depth_pixel % depth_width;
        const auto source_offset =
          static_cast<size_t>(y) * image.step + static_cast<size_t>(x) * bpp;
        if (source_offset + static_cast<size_t>(bpp) <= image.data.size()) {
          selected_depth_pixels.push_back(depth_pixel);
          selected_left = std::min(selected_left, x);
          selected_top = std::min(selected_top, y);
          selected_right = std::max(selected_right, x);
          selected_bottom = std::max(selected_bottom, y);
        }
      }
    }
    if (selected_right >= selected_left && selected_bottom >= selected_top) {
      roi = {
        selected_left,
        selected_top,
        selected_right - selected_left + 1,
        selected_bottom - selected_top + 1,
        latest_crop_.cluster_size,
        latest_crop_.highlighted_count};
      return crop_and_decimate_image(
        image, roi, output_decimation_, &selected_depth_pixels);
    }
    return black_image_like(image);
  }

  void ensure_thermal_to_depth_lookup(
    int depth_width,
    int depth_height,
    int thermal_width,
    int thermal_height)
  {
    const auto thermal_pixels = static_cast<size_t>(thermal_width * thermal_height);
    if (
      cached_depth_width_ == depth_width && cached_depth_height_ == depth_height &&
      cached_thermal_width_ == thermal_width && cached_thermal_height_ == thermal_height &&
      thermal_to_depth_pixels_.size() == thermal_pixels)
    {
      return;
    }

    const double depth_fraction_x = fov_fraction(thermal_hfov_deg_, depth_hfov_deg_) *
      thermal_scale_ * thermal_stretch_x_;
    const double depth_fraction_y = fov_fraction(thermal_vfov_deg_, depth_vfov_deg_) *
      thermal_scale_ * thermal_stretch_y_;
    const int window_width = std::max(
      latest_thermal_width_, static_cast<int>(std::round(depth_width * depth_fraction_x)));
    const int window_height = std::max(
      latest_thermal_height_, static_cast<int>(std::round(depth_height * depth_fraction_y)));
    const int window_left = static_cast<int>(
      std::round((depth_width - window_width) / 2.0 + thermal_offset_x_));
    const int window_top = static_cast<int>(
      std::round((depth_height - window_height) / 2.0 + thermal_offset_y_));
    const double inverse_window_width = 1.0 / std::max(1, window_width);
    const double inverse_window_height = 1.0 / std::max(1, window_height);

    thermal_to_depth_pixels_.assign(thermal_pixels, {});
    const auto depth_pixel_count =
      static_cast<size_t>(depth_width) * static_cast<size_t>(depth_height);
    const auto average_depth_pixels =
      (depth_pixel_count + thermal_pixels - 1) / thermal_pixels;
    for (auto & depth_pixels : thermal_to_depth_pixels_) {
      depth_pixels.reserve(average_depth_pixels);
    }
    for (int y = 0; y < depth_height; ++y) {
      const double destination_y = (y - window_top) * inverse_window_height;
      for (int x = 0; x < depth_width; ++x) {
        const double destination_x = (x - window_left) * inverse_window_width;
        double source_x = destination_x;
        double source_y = destination_y;
        if (thermal_barrel_distortion_ != 0.0) {
          const double normalized_x = destination_x * 2.0 - 1.0;
          const double normalized_y = destination_y * 2.0 - 1.0;
          const double radius_squared =
            (normalized_x * normalized_x + normalized_y * normalized_y) / 2.0;
          const double radial_scale = 1.0 + thermal_barrel_distortion_ * radius_squared;
          source_x = (normalized_x * radial_scale + 1.0) / 2.0;
          source_y = (normalized_y * radial_scale + 1.0) / 2.0;
        }
        if (source_x < 0.0 || source_x >= 1.0 || source_y < 0.0 || source_y >= 1.0) {
          continue;
        }

        const int display_x = std::clamp(
          static_cast<int>(std::floor(source_x * thermal_width)), 0, thermal_width - 1);
        const int display_y = std::clamp(
          static_cast<int>(std::floor(source_y * thermal_height)), 0, thermal_height - 1);
        const int thermal_x = flip_thermal_x_ ? thermal_width - 1 - display_x : display_x;
        const int thermal_y = flip_thermal_y_ ? thermal_height - 1 - display_y : display_y;
        const int thermal_pixel = thermal_y * thermal_width + thermal_x;
        thermal_to_depth_pixels_[static_cast<size_t>(thermal_pixel)].push_back(
          y * depth_width + x);
      }
    }
    cached_depth_width_ = depth_width;
    cached_depth_height_ = depth_height;
    cached_thermal_width_ = thermal_width;
    cached_thermal_height_ = thermal_height;
  }

  double fov_fraction(double inner_degrees, double outer_degrees) const
  {
    constexpr double k_pi = 3.14159265358979323846;
    const double inner = std::tan((inner_degrees * k_pi / 180.0) / 2.0);
    const double outer = std::tan((outer_degrees * k_pi / 180.0) / 2.0);
    return outer > 0.0 ? std::max(0.0, inner / outer) : 1.0;
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
  double depth_hfov_deg_ = 79.0;
  double depth_vfov_deg_ = 62.0;
  double thermal_hfov_deg_ = 90.0;
  double thermal_vfov_deg_ = 68.0;
  double thermal_offset_x_ = 0.0;
  double thermal_offset_y_ = 0.0;
  double thermal_scale_ = 1.0;
  double thermal_barrel_distortion_ = 0.0;
  double thermal_stretch_x_ = 0.8;
  double thermal_stretch_y_ = 0.9;
  bool flip_thermal_x_ = true;
  bool flip_thermal_y_ = false;
  bool passthrough_when_no_region_ = true;
  bool enabled_ = true;
  int output_decimation_ = 2;

  CropRegion latest_crop_;
  std::vector<uint8_t> latest_thermal_mask_;
  bool have_crop_ = false;
  int latest_thermal_width_ = 32;
  int latest_thermal_height_ = 24;
  int cached_depth_width_ = 0;
  int cached_depth_height_ = 0;
  int cached_thermal_width_ = 0;
  int cached_thermal_height_ = 0;
  std::vector<std::vector<int>> thermal_to_depth_pixels_;
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

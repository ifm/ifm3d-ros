/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021 ifm electronic, gmbh
 */

#include <ifm3d_ros_driver/camera_nodelet.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <ifm3d_ros_msgs/Config.h>
#include <ifm3d_ros_msgs/Dump.h>
#include <ifm3d_ros_msgs/Extrinsics.h>
#include <ifm3d_ros_msgs/SoftOff.h>
#include <ifm3d_ros_msgs/SoftOn.h>
#include <ifm3d_ros_msgs/Trigger.h>

// #include <ifm3d/contrib/nlohmann/json.hpp>
#include <ifm3d/device/json.hpp>


using namespace std::chrono_literals;
// This function formats the timestamps for proper display
// a.k.a converts to local time
std::string formatTimestamp(ifm3d::TimePointT timestamp)
{
  using namespace std::chrono;
  std::time_t time = std::chrono::system_clock::to_time_t(
    std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      timestamp));

  milliseconds milli = duration_cast<milliseconds>(
    timestamp.time_since_epoch() -
    duration_cast<seconds>(timestamp.time_since_epoch()));

  std::ostringstream s;
  s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << ":"
    << std::setw(3) << std::setfill('0') << milli.count();

  return s.str();
}

sensor_msgs::Image ifm3d_to_ros_image(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                            // image.end() don't have const overloads.
                                      const std_msgs::Header& header, const std::string& logger)
{
  static constexpr auto max_pixel_format = static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32F3);
  static auto image_format_info = [] {
    auto image_format_info = std::array<std::string, max_pixel_format + 1>{};

    {
      using namespace ifm3d;
      using namespace sensor_msgs::image_encodings;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8U)] = TYPE_8UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8S)] = TYPE_8SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U)] = TYPE_16UC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16S)] = TYPE_16SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32U)] = "32UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32S)] = TYPE_32SC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F)] = TYPE_32FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64U)] = "64UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64F)] = TYPE_64FC1;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U2)] = TYPE_16UC2;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F3)] = TYPE_32FC3;
    }

    return image_format_info;
  }();

  const auto format = static_cast<std::size_t>(image.dataFormat());

  sensor_msgs::Image result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = 0;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (format >= max_pixel_format)
  {
    ROS_ERROR_NAMED(logger, "Pixel format out of range (%ld >= %ld)", format, max_pixel_format);
    return result;
  }

  result.encoding = image_format_info.at(format);
  result.step = result.width * sensor_msgs::image_encodings::bitDepth(image_format_info.at(format)) / 8;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.step * result.height));

  if (result.encoding.empty())
  {
    ROS_WARN_NAMED(logger, "Can't handle encoding %ld (32U == %ld, 64U == %ld)", format,
                   static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32U),
                   static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_64U));
    result.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  }

  return result;
}

sensor_msgs::Image ifm3d_to_ros_image(ifm3d::Buffer&& image, const std_msgs::Header& header, const std::string& logger)
{
  return ifm3d_to_ros_image(image, header, logger);
}

sensor_msgs::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer& image,  // Need non-const image because
                                                                                 // image.begin(), image.end()
                                                                                 // don't have const overloads.
                                                           const std_msgs::Header& header,
                                                           const std::string& format,  // "jpeg" or "png"
                                                           const std::string& logger)
{
  sensor_msgs::CompressedImage result{};
  result.header = header;
  result.format = format;

  {
    const auto dataFormat = image.dataFormat();
    if (dataFormat != ifm3d::pixel_format::FORMAT_8S && dataFormat != ifm3d::pixel_format::FORMAT_8U)
    {
      ROS_ERROR_NAMED(logger, "Invalid data format for %s data (%ld)", format.c_str(),
                      static_cast<std::size_t>(dataFormat));
      return result;
    }
  }

  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), image.width() * image.height()));
  return result;
}

sensor_msgs::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer&& image, const std_msgs::Header& header,
                                                           const std::string& format, const std::string& logger)
{
  return ifm3d_to_ros_compressed_image(image, header, format, logger);
}

sensor_msgs::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                  // image.end() don't have const overloads.
                                            const std_msgs::Header& header, const std::string& logger)
{
  sensor_msgs::PointCloud2 result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = false;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (image.dataFormat() != ifm3d::pixel_format::FORMAT_32F3 && image.dataFormat() != ifm3d::pixel_format::FORMAT_32F)
  {
    ROS_ERROR_NAMED(logger, "Unsupported pixel format %ld for point cloud",
                    static_cast<std::size_t>(image.dataFormat()));
    return result;
  }

  sensor_msgs::PointField x_field{};
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::PointField y_field{};
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::PointField::FLOAT32;
  y_field.count = 1;

  sensor_msgs::PointField z_field{};
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::PointField::FLOAT32;
  z_field.count = 1;

  result.fields = {
    x_field,
    y_field,
    z_field,
  };

  result.point_step = result.fields.size() * sizeof(float);
  result.row_step = result.point_step * result.width;
  result.is_dense = true;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.row_step * result.height));

  return result;
}

sensor_msgs::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer&& image, const std_msgs::Header& header,
                                            const std::string& logger)
{
  return ifm3d_to_ros_cloud(image, header, logger);
}

using json = ifm3d::json;
namespace enc = sensor_msgs::image_encodings;

void ifm3d_ros::CameraNodelet::onInit()
{
  std::string nn = this->getName();
  NODELET_DEBUG_STREAM("onInit(): " << nn);

  this->np_ = getMTPrivateNodeHandle();
  this->it_.reset(new image_transport::ImageTransport(this->np_));

  //
  // parse data out of the parameter server
  //
  // NOTE: AFAIK, there is no way to get an unsigned int type out of the ROS
  // parameter server.
  //

  int schema_mask = { ifm3d_legacy::IMG_RDIS | ifm3d_legacy::IMG_AMP | ifm3d_legacy::IMG_RAMP | ifm3d_legacy::IMG_CART };
  int xmlrpc_port;
  int pcic_port;
  std::string frame_id_base;
  const ifm3d::FrameGrabber::BufferList DEFAULT_SCHEMA_MASK = {
                              ifm3d::buffer_id::XYZ,
                              ifm3d::buffer_id::CONFIDENCE_IMAGE,
                              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
                              ifm3d::buffer_id::RADIAL_DISTANCE_NOISE,
                              ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
                              ifm3d::buffer_id::AMPLITUDE_IMAGE,
                              ifm3d::buffer_id::EXTRINSIC_CALIB,
                              ifm3d::buffer_id::JPEG_IMAGE
                            };

  if ((nn.size() > 0) && (nn.at(0) == '/'))
  {
    frame_id_base = nn.substr(1);
  }
  else
  {
    frame_id_base = nn;
  }

  this->np_.param("ip", this->camera_ip_, ifm3d::DEFAULT_IP);
  NODELET_INFO("IP default: %s, current %s", ifm3d::DEFAULT_IP.c_str(), this->camera_ip_.c_str());

  this->np_.param("xmlrpc_port", xmlrpc_port, (int)ifm3d::DEFAULT_XMLRPC_PORT);
  this->np_.param("pcic_port", pcic_port, (int)ifm3d::DEFAULT_PCIC_PORT);
  NODELET_INFO("pcic port check: current %d, default %d", pcic_port, ifm3d::DEFAULT_PCIC_PORT);

  this->np_.param("password", this->password_, ifm3d::DEFAULT_PASSWORD);
  // this->np_.param<ifm3d::FrameGrabber::BufferList>("schema_mask", this->schema_mask_, DEFAULT_SCHEMA_MASK);

  this->np_.param("schema_mask", schema_mask);
  this->np_.param("timeout_millis", this->timeout_millis_, 500);
  this->np_.param("timeout_tolerance_secs", this->timeout_tolerance_secs_, 5.0);
  this->np_.param("assume_sw_triggered", this->assume_sw_triggered_, false);
  this->np_.param("soft_on_timeout_millis", this->soft_on_timeout_millis_, 500);
  this->np_.param("soft_on_timeout_tolerance_secs", this->soft_on_timeout_tolerance_secs_, 5.0);
  this->np_.param("soft_off_timeout_millis", this->soft_off_timeout_millis_, 500);
  this->np_.param("soft_off_timeout_tolerance_secs", this->soft_off_timeout_tolerance_secs_, 600.0);
  this->np_.param("frame_latency_thresh", this->frame_latency_thresh_, 60.0f);
  this->np_.param("frame_id_base", frame_id_base, frame_id_base);

  this->xmlrpc_port_ = static_cast<std::uint16_t>(xmlrpc_port);
  this->schema_mask_ = static_cast<std::uint16_t>(schema_mask);
  this->schema_mask_default_ = DEFAULT_SCHEMA_MASK;
  this->pcic_port_ = static_cast<std::uint16_t>(pcic_port);

  NODELET_DEBUG_STREAM("setup ros node parameters finished");

  this->frame_id_ = frame_id_base + "_link";
  this->optical_frame_id_ = frame_id_base + "_optical_link";

  //-------------------
  // Published topics
  //-------------------
  this->cloud_pub_ = this->np_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  this->distance_pub_ = this->it_->advertise("distance", 1);
  this->distance_noise_pub_ = this->it_->advertise("distance_noise", 1);
  this->amplitude_pub_ = this->it_->advertise("amplitude", 1);
  this->raw_amplitude_pub_ = this->it_->advertise("raw_amplitude", 1);
  this->conf_pub_ = this->it_->advertise("confidence", 1);
  this->rgb_image_pub_ = this->np_.advertise<sensor_msgs::CompressedImage>("rgb_image/compressed", 1);

  // we latch the unit vectors
  this->uvec_pub_ = this->np_.advertise<sensor_msgs::Image>("unit_vectors", 1, true);

  this->extrinsics_pub_ = this->np_.advertise<ifm3d_ros_msgs::Extrinsics>("extrinsics", 1);
  NODELET_DEBUG_STREAM("after advertising the publishers");
  //---------------------
  // Advertised Services
  //---------------------
  this->dump_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Dump::Request, ifm3d_ros_msgs::Dump::Response>(
      "Dump", std::bind(&CameraNodelet::Dump, this, std::placeholders::_1, std::placeholders::_2));

  this->config_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Config::Request, ifm3d_ros_msgs::Config::Response>(
      "Config", std::bind(&CameraNodelet::Config, this, std::placeholders::_1, std::placeholders::_2));

  this->trigger_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Trigger::Request, ifm3d_ros_msgs::Trigger::Response>(
      "Trigger", std::bind(&CameraNodelet::Trigger, this, std::placeholders::_1, std::placeholders::_2));

  this->soft_off_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::SoftOff::Request, ifm3d_ros_msgs::SoftOff::Response>(
      "SoftOff", std::bind(&CameraNodelet::SoftOff, this, std::placeholders::_1, std::placeholders::_2));

  this->soft_on_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::SoftOn::Request, ifm3d_ros_msgs::SoftOn::Response>(
      "SoftOn", std::bind(&CameraNodelet::SoftOn, this, std::placeholders::_1, std::placeholders::_2));

  NODELET_DEBUG_STREAM("after advertise service");
  //----------------------------------
  // Fire off our main publishing loop
  //----------------------------------
  this->publoop_timer_ = this->np_.createTimer(
      ros::Duration(.001), [this](const ros::TimerEvent& t) { this->Run(); },
      true);  // oneshot timer
}

bool ifm3d_ros::CameraNodelet::Dump(ifm3d_ros_msgs::Dump::Request& req, ifm3d_ros_msgs::Dump::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  try
  {
    json j = this->cam_->ToJSON();
    res.config = j.dump();
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    NODELET_WARN_STREAM(ex.what());
  }
  catch (const std::exception& std_ex)
  {
    res.status = -1;
    NODELET_WARN_STREAM(std_ex.what());
  }
  catch (...)
  {
    res.status = -2;
  }

  if (res.status != 0)
  {
    NODELET_WARN_STREAM("Dump: " << res.status);
  }

  return true;
}

bool ifm3d_ros::CameraNodelet::Config(ifm3d_ros_msgs::Config::Request& req, ifm3d_ros_msgs::Config::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  try
  {
    this->cam_->FromJSON(json::parse(req.json));
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
  }
  catch (const std::exception& std_ex)
  {
    res.status = -1;
    res.msg = std_ex.what();
  }
  catch (...)
  {
    res.status = -2;
    res.msg = "Unknown error in `Config'";
  }

  if (res.status != 0)
  {
    NODELET_WARN_STREAM("Config: " << res.status << " - " << res.msg);
  }

  return true;
}

bool ifm3d_ros::CameraNodelet::Trigger(ifm3d_ros_msgs::Trigger::Request& req, ifm3d_ros_msgs::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "Software trigger is currently not implemented";

  try
  {
    this->fg_->SWTrigger();
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
  }

  NODELET_WARN_STREAM("Triggering a camera head is currently not implemented - will follow");
  return true;
}

// this is a dummy method for the moment:  the idea of applications is not supported for the O3RCamera
// we keep this in to possibly keep it comparable / interoperable with the ROS wrappers for other ifm cameras
bool ifm3d_ros::CameraNodelet::SoftOff(ifm3d_ros_msgs::SoftOff::Request& req, ifm3d_ros_msgs::SoftOff::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  int port_arg = -1;

  try
  {
    port_arg = static_cast<int>(this->pcic_port_) % 50010;

    // Configure the device from a json string
    this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"IDLE\"}}}");

    this->assume_sw_triggered_ = false;
    this->timeout_millis_ = this->soft_on_timeout_millis_;
    this->timeout_tolerance_secs_ = this->soft_on_timeout_tolerance_secs_;
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
    return false;
  }

  NODELET_WARN_STREAM("The concept of applications is not available for the O3R - we use IDLE and RUN states instead");
  res.msg = "{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"IDLE\"}}}";

  return true;
}

// this is a dummy method for the moment:  the idea of applications is not supported for the O3RCamera
// we keep this in to possibly keep it comparable / interoperable with the ROS wrappers for other ifm cameras
bool ifm3d_ros::CameraNodelet::SoftOn(ifm3d_ros_msgs::SoftOn::Request& req, ifm3d_ros_msgs::SoftOn::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  int port_arg = -1;

  try
  {
    port_arg = static_cast<int>(this->pcic_port_) % 50010;

    // try getting a current configuration as an ifm3d dump
    // this way a a-priori test before setting the state can be tested
    // try
    // {
    //   json j = this->cam_->ToJSON();
    // }
    // catch (const ifm3d::Error& ex)
    // {
    //   NODELET_WARN_STREAM(ex.code());
    //   NODELET_WARN_STREAM(ex.what());
    // }
    // catch (const std::exception& std_ex)
    //   {
    //     NODELET_WARN_STREAM(std_ex.what());
    // }

    // Configure the device from a json string
    this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"RUN\"}}}");

    this->assume_sw_triggered_ = false;
    this->timeout_millis_ = this->soft_on_timeout_millis_;
    this->timeout_tolerance_secs_ = this->soft_on_timeout_tolerance_secs_;
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
    return false;
  }

  NODELET_WARN_STREAM("The concept of applications is not available for the O3R - we use IDLE and RUN states instead");
  res.msg = "{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"RUN\"}}}";

  return true;
}

bool ifm3d_ros::CameraNodelet::InitStructures(std::uint16_t pcic_port)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  bool retval = false;

  try
  {
    NODELET_INFO_STREAM("Running dtors...");
    this->fg_.reset();
    this->cam_.reset();

    NODELET_INFO_STREAM("Initializing camera...");
    this->cam_ = ifm3d::Device::MakeShared(this->camera_ip_, this->xmlrpc_port_);
    ros::Duration(1.0).sleep();

    NODELET_INFO_STREAM("Initializing framegrabber...");
    this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, this->pcic_port_);
    NODELET_INFO("Nodelet argument: %d", (int)this->pcic_port_);

    retval = true;
  }
  catch (const ifm3d::Error& ex)
  {
    NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
    this->fg_.reset();
    this->cam_.reset();
    retval = false;
  }

  return retval;
}

void ifm3d_ros::CameraNodelet::Callback(ifm3d::Frame::Ptr frame){
    //
    // Pull out all the wrapped images so that we can release the "GIL"
    // while publishing
    //
    std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);
    lock.lock();
    ifm3d::Buffer xyz_img;
    ifm3d::Buffer confidence_img;
    ifm3d::Buffer distance_img;
    ifm3d::Buffer distance_noise_img;
    ifm3d::Buffer amplitude_img;
    ifm3d::Buffer raw_amplitude_img;
    ifm3d::Buffer extrinsics;
    ifm3d::Buffer rgb_img;

    NODELET_DEBUG_STREAM("start getting data");
    try
    {
      xyz_img =frame->GetBuffer(ifm3d::buffer_id::XYZ);
      confidence_img = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);
      distance_img = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
      distance_noise_img = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE);
      amplitude_img = frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE);
      raw_amplitude_img = frame->GetBuffer(ifm3d::buffer_id::AMPLITUDE_IMAGE);
      extrinsics = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);
      rgb_img = frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE);
      this->last_frame_time_ = frame->TimeStamps()[0];

    }
    catch (const ifm3d::Error& ex)
    {
      NODELET_WARN_STREAM(ex.what());
    }
    catch (const std::exception& std_ex)
    {
      NODELET_WARN_STREAM(std_ex.what());
    }
    NODELET_DEBUG_STREAM("finished getting data");

    lock.unlock();

    //
    // Now, do the publishing
    //

    // Timestamps:
    this->head.stamp = ros::Time(
      std::chrono::duration_cast<std::chrono::duration<double,
      std::ratio<1>>>(frame->TimeStamps()[0].time_since_epoch()).count()
      );
    if ((ros::Time::now() - this->head.stamp) > ros::Duration(this->frame_latency_thresh_))
    {
      NODELET_INFO_ONCE("Camera's time and client's time are not synced");
      this->head.stamp = ros::Time::now();
    }

    if (frame->HasBuffer(ifm3d::buffer_id::JPEG_IMAGE))
    {
      this->rgb_image_pub_.publish(ifm3d_to_ros_compressed_image(rgb_img, optical_head, "jpeg", getName()));
      NODELET_DEBUG_STREAM("after publishing rgb image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE))
    {
      this->distance_pub_.publish(ifm3d_to_ros_image(distance_img, optical_head, getName()));
      NODELET_DEBUG_STREAM("after publishing distance image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE))
    {
      this->conf_pub_.publish(ifm3d_to_ros_image(confidence_img, optical_head, getName()));
      NODELET_DEBUG_STREAM("after publishing confidence image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE))
    {
      this->amplitude_pub_.publish(ifm3d_to_ros_image(amplitude_img, optical_head, getName()));
      NODELET_DEBUG_STREAM("after publishing amplitude image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::AMPLITUDE_IMAGE))
    {
      this->raw_amplitude_pub_.publish(ifm3d_to_ros_image(raw_amplitude_img, optical_head, getName()));
      NODELET_DEBUG_STREAM("after publishing norm amplitude image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::XYZ))
    {
      this->cloud_pub_.publish(ifm3d_to_ros_cloud(xyz_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing point cloud image");
    }

    if (frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE))
    {
      this->distance_noise_pub_.publish(ifm3d_to_ros_image(distance_noise_img, optical_head, getName()));
      NODELET_DEBUG_STREAM("after publishing distance noise image");
    }

    // // Confidence image is invariant - no need to check the mask
    // this->conf_pub_.publish(ifm3d_to_ros_image(confidence_img, optical_head, getName()));
    // NODELET_DEBUG_STREAM("after publishing confidence image");

    // if (std::find(this->schema_mask_.begin(), this->schema_mask_.end(), ifm3d::buffer_id::XYZ) != this->schema_mask_.end())
    // // if ((this->schema_mask_ & ifm3d::buffer_id::XYZ) == ifm3d::buffer_id::XYZ)
    // {
    //   this->cloud_pub_.publish(ifm3d_to_ros_cloud(xyz_img, head, getName()));
    //   NODELET_DEBUG_STREAM("after publishing xyz image");
    // }

    // if (std::find(this->schema_mask_.begin(), this->schema_mask_.end(), ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE) != this->schema_mask_.end())
    // // if ((this->schema_mask_ & ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE) == ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE)
    // {
    //   this->distance_pub_.publish(ifm3d_to_ros_image(distance_img, optical_head, getName()));
    //   NODELET_DEBUG_STREAM("after publishing distance image");
    // }


    // if (std::find(this->schema_mask_.begin(), this->schema_mask_.end(), ifm3d::buffer_id::RADIAL_DISTANCE_NOISE) != this->schema_mask_.end())
    // // if ((this->schema_mask_ & ifm3d::buffer_id::RADIAL_DISTANCE_NOISE) == ifm3d::buffer_id::RADIAL_DISTANCE_NOISE)
    // {
    //   this->distance_noise_pub_.publish(ifm3d_to_ros_image(distance_noise_img, optical_head, getName()));
    //   NODELET_DEBUG_STREAM("after publishing distance noise image");
    // }

    // if (std::find(this->schema_mask_.begin(), this->schema_mask_.end(), ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE) != this->schema_mask_.end())
    // // if ((this->schema_mask_ & ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE) == ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE)
    // {
    //   this->amplitude_pub_.publish(ifm3d_to_ros_image(amplitude_img, optical_head, getName()));
    //   NODELET_DEBUG_STREAM("after publishing amplitude image");
    // }

    // if (std::find(this->schema_mask_.begin(), this->schema_mask_.end(), ifm3d::buffer_id::AMPLITUDE_IMAGE) != this->schema_mask_.end())
    // // if ((this->schema_mask_ & ifm3d::buffer_id::AMPLITUDE_IMAGE) == ifm3d::buffer_id::AMPLITUDE_IMAGE)
    // {
    //   this->raw_amplitude_pub_.publish(ifm3d_to_ros_image(raw_amplitude_img, optical_head, getName()));
    //   NODELET_DEBUG_STREAM("Raw amplitude image publisher is a dummy publisher - data will be added soon");
    //   NODELET_DEBUG_STREAM("after publishing raw amplitude image");
    // }

    // // The 2D is not yet settable in the schema mask: publish all the time

    // if (rgb_img.height() * rgb_img.width() > 0)
    // {
    //   this->rgb_image_pub_.publish(ifm3d_to_ros_compressed_image(rgb_img, optical_head, "jpeg", getName()));
    //   NODELET_DEBUG_STREAM("after publishing rgb image");
    // }



    //
    // publish extrinsics
    //
    NODELET_DEBUG_STREAM("start publishing extrinsics");
    ifm3d_ros_msgs::Extrinsics extrinsics_msg;
    extrinsics_msg.header = optical_head;
    try
    {
      ifm3d::Buffer_<float> ext = extrinsics;
      extrinsics_msg.tx = ext.at(0);
      extrinsics_msg.ty = ext.at(1);
      extrinsics_msg.tz = ext.at(2);
      extrinsics_msg.rot_x = ext.at(3);
      extrinsics_msg.rot_y = ext.at(4);
      extrinsics_msg.rot_z = ext.at(5);
    }
    catch (const std::out_of_range& ex)
    {
      NODELET_WARN("out-of-range error fetching extrinsics");
    }
    this->extrinsics_pub_.publish(extrinsics_msg);
}
// this is the helper function for retrieving complete pcic frames
bool ifm3d_ros::CameraNodelet::StartStream()
{
  bool retval = false;
  NODELET_DEBUG_STREAM("Start streaming frames");
  try
  {
    fg_->Start(this->schema_mask_default_);
    NODELET_INFO_STREAM("Framegabbber initialized with default schema mask");

    // XXX: need to implement a nice strategy for getting the actual times
    // from the camera which are registered to the frame data in the image
    // buffer.

    bool got_uvec = false;
    NODELET_INFO_STREAM("prepare header");
    this->head = std_msgs::Header();
    this->head.frame_id = this->frame_id_;
    this->head.stamp = ros::Time::now();

    this->optical_head = std_msgs::Header();
    this->optical_head.stamp = head.stamp;
    this->optical_head.frame_id = this->optical_frame_id_;

    fg_->OnNewFrame(std::bind(&ifm3d_ros::CameraNodelet::Callback, this, std::placeholders::_1));
  }
  catch (const ifm3d::Error& ex)
  {
    NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
    retval = false;
  }

  return retval;
}

void ifm3d_ros::CameraNodelet::Run()
{
  std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);

  NODELET_DEBUG_STREAM("in Run");

  // We need to account for the case of when the nodelet is being started prior
  // to the camera being plugged in.

  while (ros::ok() && (!this->InitStructures(this->pcic_port_)))
  {
    NODELET_WARN_STREAM("Could not initialize pixel stream!");
    ros::Duration(1.0).sleep();
  }

  NODELET_DEBUG_STREAM("after initializing the buffers");

  this->StartStream();
  NODELET_INFO_ONCE("Started the camera stream");

  while (ros::ok())
  {
    // if ((ros::Time::now() - ros::Time().fromNSec(std::chrono::system_clock::to_time_t(
    // std::chrono::time_point_cast<std::chrono::system_clock::duration>(
    //   this->last_frame_time_)))) > ros::Duration().fromSec(this->timeout_tolerance_secs_))
    // {
    //   if (!this->assume_sw_triggered_)
    //   {
    //     NODELET_WARN_STREAM("Timeout waiting for camera!");
    //     NODELET_WARN_STREAM("Attempting to restart framegrabber...");
    //     while (!this->InitStructures(this->pcic_port_))
    //     {
    //       NODELET_WARN_STREAM("Could not re-initialize pixel stream!");
    //       ros::Duration(1.0).sleep();
    //     }
    //   }
    // }
    // else
    // {
    //   ros::Duration(.001).sleep();
    // }

    continue;
  }

  fg_->Stop();
}  // end: Run()

PLUGINLIB_EXPORT_CLASS(ifm3d_ros::CameraNodelet, nodelet::Nodelet)

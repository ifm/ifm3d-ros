/*
* SPDX-License-Identifier: Apache-2.0
* Copyright (C) 2021 ifm electronic, gmbh
*/


#include <ifm3d/camera_nodelet.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <ifm3d/camera/camera_base.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d/Config.h>
#include <ifm3d/Dump.h>
#include <ifm3d/Extrinsics.h>
#include <ifm3d/SoftOff.h>
#include <ifm3d/SoftOn.h>
#include <ifm3d/Trigger.h>

#include <ifm3d/contrib/nlohmann/json.hpp>

using json = nlohmann::json;
namespace enc = sensor_msgs::image_encodings;

void
ifm3d_ros::CameraNodelet::onInit()
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
  int schema_mask;
  int xmlrpc_port;
  int pcic_port;
  std::string frame_id_base;

  if ((nn.size() > 0) && (nn.at(0) == '/'))
    {
      frame_id_base = nn.substr(1);
    }
  else
    {
      frame_id_base = nn;
    }

  this->np_.param("ip", this->camera_ip_, ifm3d::DEFAULT_IP);
  ROS_INFO("IP default: %s, current %s", ifm3d::DEFAULT_IP.c_str(), this->camera_ip_.c_str());

  this->np_.param("xmlrpc_port", xmlrpc_port, (int) ifm3d::DEFAULT_XMLRPC_PORT);
  this->np_.param("pcic_port", pcic_port, (int) ifm3d::DEFAULT_PCIC_PORT);
  ROS_INFO("pcic port check: current %d, default %d", pcic_port, ifm3d::DEFAULT_PCIC_PORT);

  this->np_.param("password", this->password_, ifm3d::DEFAULT_PASSWORD);
  this->np_.param("schema_mask", schema_mask, (int) ifm3d::DEFAULT_SCHEMA_MASK);
  this->np_.param("timeout_millis", this->timeout_millis_, 500);
  this->np_.param("timeout_tolerance_secs", this->timeout_tolerance_secs_, 5.0);
  this->np_.param("assume_sw_triggered", this->assume_sw_triggered_, false);
  this->np_.param("soft_on_timeout_millis", this->soft_on_timeout_millis_, 500);
  this->np_.param("soft_on_timeout_tolerance_secs",
                  this->soft_on_timeout_tolerance_secs_, 5.0);
  this->np_.param("soft_off_timeout_millis",
                  this->soft_off_timeout_millis_, 500);
  this->np_.param("soft_off_timeout_tolerance_secs",
                  this->soft_off_timeout_tolerance_secs_, 600.0);
  this->np_.param("frame_latency_thresh", this->frame_latency_thresh_, 60.0f);
  this->np_.param("frame_id_base", frame_id_base, frame_id_base);

  this->xmlrpc_port_ = static_cast<std::uint16_t>(xmlrpc_port);
  this->schema_mask_ = static_cast<std::uint16_t>(schema_mask);
  this->pcic_port_ = static_cast<std::uint16_t>(pcic_port);

  NODELET_DEBUG_STREAM("setup ros node parameters finished");

  this->frame_id_ = frame_id_base + "_link";
  this->optical_frame_id_ = frame_id_base + "_optical_link";

  //-------------------
  // Published topics
  //-------------------
  this->cloud_pub_ =
    this->np_.advertise<pcl::PointCloud<ifm3d::PointT> >("cloud", 1);
  this->distance_pub_ = this->it_->advertise("distance", 1);
  // this->distance_noise_pub_ = this->it_->advertise("distance_noise", 1);
  this->amplitude_pub_ = this->it_->advertise("amplitude", 1);
  this->raw_amplitude_pub_ = this->it_->advertise("raw_amplitude", 1);
  this->conf_pub_ = this->it_->advertise("confidence", 1);
  this->good_bad_pub_ = this->it_->advertise("good_bad_pixels", 1);
  this->xyz_image_pub_ = this->it_->advertise("xyz_image", 1);
  this->gray_image_pub_ = this->it_->advertise("gray_image", 1);
  this->rgb_image_pub_ = this->it_->advertise("rgb_image", 1);

  // we latch the unit vectors
  this->uvec_pub_ =
    this->np_.advertise<sensor_msgs::Image>("unit_vectors", 1, true);

  this->extrinsics_pub_ =
    this->np_.advertise<ifm3d::Extrinsics>("extrinsics", 1);
  NODELET_DEBUG_STREAM("after advertising the publishers");
  //---------------------
  // Advertised Services
  //---------------------
  this->dump_srv_ =
    this->np_.advertiseService<ifm3d::Dump::Request, ifm3d::Dump::Response>
    ("Dump", std::bind(&CameraNodelet::Dump, this,
                       std::placeholders::_1,
                       std::placeholders::_2));

  this->config_srv_ =
    this->np_.advertiseService<ifm3d::Config::Request, ifm3d::Config::Response>
    ("Config", std::bind(&CameraNodelet::Config, this,
                         std::placeholders::_1,
                         std::placeholders::_2));

  this->trigger_srv_ =
    this->np_.advertiseService<ifm3d::Trigger::Request,
                               ifm3d::Trigger::Response>
    ("Trigger", std::bind(&CameraNodelet::Trigger, this,
                          std::placeholders::_1,
                          std::placeholders::_2));

  this->soft_off_srv_ =
    this->np_.advertiseService<ifm3d::SoftOff::Request,
                               ifm3d::SoftOff::Response>
    ("SoftOff", std::bind(&CameraNodelet::SoftOff, this,
                          std::placeholders::_1,
                          std::placeholders::_2));

  this->soft_on_srv_ =
    this->np_.advertiseService<ifm3d::SoftOn::Request,
                               ifm3d::SoftOn::Response>
    ("SoftOn", std::bind(&CameraNodelet::SoftOn, this,
                          std::placeholders::_1,
                          std::placeholders::_2));

  NODELET_DEBUG_STREAM("after advertise service");
  //----------------------------------
  // Fire off our main publishing loop
  //----------------------------------
  this->publoop_timer_ =
    this->np_.createTimer(ros::Duration(.001),
                          [this](const ros::TimerEvent& t)
                          { this->Run(); },
                          true); // oneshot timer
}

bool
ifm3d_ros::CameraNodelet::Dump(ifm3d::Dump::Request& req,
                               ifm3d::Dump::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  try
    {
      json j = this->cam_->ToJSON();
      res.config = j.dump();
    }
  catch (const ifm3d::error_t& ex)
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

bool
ifm3d_ros::CameraNodelet::Config(ifm3d::Config::Request& req,
                                 ifm3d::Config::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  try
    {
      this->cam_->FromJSON(req.json);
    }
  catch (const ifm3d::error_t& ex)
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

bool
ifm3d_ros::CameraNodelet::Trigger(ifm3d::Trigger::Request& req,
                                  ifm3d::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "Software trigger is currently not implemented";

  try
    {
      this->fg_->SWTrigger();
    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
    }

  NODELET_WARN_STREAM("Triggering a camera head is currently not implemented - will follow");
  return true;
}

// this is a dummy method for the moment:  the idea of applications is not supported for the O3RCamera
// we keep this in to possibly keep it comparable / interoperable with the ROS wrappers for other ifm cameras
bool
ifm3d_ros::CameraNodelet::SoftOff(ifm3d::SoftOff::Request& req,
                                  ifm3d::SoftOff::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  int port_arg = -1;

  try
    {
      port_arg = static_cast<int> (this->pcic_port_) % 50010;

      // Configure the device from a json string
      this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"IDLE\"}}}");

      this->assume_sw_triggered_ = false;
      this->timeout_millis_ = this->soft_on_timeout_millis_;
      this->timeout_tolerance_secs_ =
      this->soft_on_timeout_tolerance_secs_;

    }
  catch (const ifm3d::error_t& ex)
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
bool
ifm3d_ros::CameraNodelet::SoftOn(ifm3d::SoftOn::Request& req,
                                 ifm3d::SoftOn::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  int port_arg = -1;

  try
    {
      port_arg = static_cast<int> (this->pcic_port_) % 50010;

      // try getting a current configuration as an ifm3d dump
      // this way a a-priori test before setting the state can be tested
      // try
      // {
      //   json j = this->cam_->ToJSON();
      // }
      // catch (const ifm3d::error_t& ex)
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
      this->timeout_tolerance_secs_ =
      this->soft_on_timeout_tolerance_secs_;

    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
      return false;
    }

  NODELET_WARN_STREAM("The concept of applications is not available for the O3R - we use IDLE and RUN states instead");
  res.msg = "{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"RUN\"}}}";

  return true;
}

bool
ifm3d_ros::CameraNodelet::InitStructures(std::uint16_t mask, std::uint16_t pcic_port)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  bool retval = false;

  try
    {
      NODELET_INFO_STREAM("Running dtors...");
      this->im_.reset();
      this->fg_.reset();
      this->cam_.reset();

      NODELET_INFO_STREAM("Initializing camera...");
      this->cam_ = ifm3d::CameraBase::MakeShared();
      // this->cam_ = ifm3d::CameraBase::MakeShared(this->camera_ip_, this->xmlrpc_port_);
      // this->cam_ = std::make_shared<ifm3d::CameraBase>(this->camera_ip_, this->xmlrpc_port_);
      ros::Duration(1.0).sleep();

      NODELET_INFO_STREAM("Initializing framegrabber...");
      this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, mask, this->pcic_port_);
      ROS_INFO("Nodelet arguments: %d, %d", (int) mask, (int) this->pcic_port_);

      NODELET_INFO_STREAM("Initializing image buffer...");
      this->im_ = std::make_shared<ifm3d::ImageBuffer>();

      retval = true;
    }
  catch (const ifm3d::error_t& ex)
    {
      NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
      this->im_.reset();
      this->fg_.reset();
      this->cam_.reset();
      retval = false;
    }

  return retval;
}

// this is the helper function for retrieving complete pcic frames
bool
ifm3d_ros::CameraNodelet::AcquireFrame()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  bool retval = false;
  NODELET_DEBUG_STREAM("try receiving data via fg WaitForFrame");
  try
    {
      retval = this->fg_->WaitForFrame(this->im_.get(), this->timeout_millis_);
    }
  catch (const ifm3d::error_t& ex)
    {
      NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
      retval = false;
    }

  return retval;
}

void
ifm3d_ros::CameraNodelet::Run()
{
  std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);

  NODELET_DEBUG_STREAM("in Run");

  // We need to account for the case of when the nodelet is being started prior
  // to the camera being plugged in.

  while (ros::ok() && (! this->InitStructures(ifm3d::IMG_UVEC, this->pcic_port_)))
    {
      NODELET_WARN_STREAM("Could not initialize pixel stream!");
      ros::Duration(1.0).sleep();
    }

  pcl::PointCloud<ifm3d::PointT>::Ptr
    cloud(new pcl::PointCloud<ifm3d::PointT>());

  cv::Mat confidence_img;
  cv::Mat distance_img;
  // cv::Mat distance_noise_img;
  cv::Mat amplitude_img;
  cv::Mat xyz_img;
  cv::Mat raw_amplitude_img;
  cv::Mat good_bad_pixels_img;
  cv::Mat gray_img;
  cv::Mat rgb_img;


  NODELET_DEBUG_STREAM("after initializing the opencv buffers");
  std::vector<float> extrinsics(6);

  // XXX: need to implement a nice strategy for getting the actual times
  // from the camera which are registered to the frame data in the image
  // buffer.
  ros::Time last_frame = ros::Time::now();
  bool got_uvec = false;

  while (ros::ok())
    {
      if (! this->AcquireFrame())
        {
          if (! this->assume_sw_triggered_)
            {
              NODELET_WARN_STREAM("Timeout waiting for camera!");
            }
          else
            {
              ros::Duration(.001).sleep();
            }

          if ((ros::Time::now() - last_frame).toSec() >
              this->timeout_tolerance_secs_)
            {
              NODELET_WARN_STREAM("Attempting to restart framegrabber...");
              while (! this->InitStructures(got_uvec
                                            ? this->schema_mask_
                                            : ifm3d::IMG_UVEC, this->pcic_port_))
                {
                  NODELET_WARN_STREAM("Could not re-initialize pixel stream!");
                  ros::Duration(1.0).sleep();
                }

              last_frame = ros::Time::now();
            }

          continue;
        }

      last_frame = ros::Time::now();

      NODELET_DEBUG_STREAM("prepare header");
      std_msgs::Header head = std_msgs::Header();
      head.frame_id = this->frame_id_;
      head.stamp = ros::Time(
        std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>
        (this->im_->TimeStamp().time_since_epoch()).count());
      if ((ros::Time::now() - head.stamp) >
          ros::Duration(this->frame_latency_thresh_))
        {
          ROS_INFO_ONCE("Camera's time and client's time are not synced");
          head.stamp = ros::Time::now();
        }
      NODELET_DEBUG_STREAM("in header, before setting header to msgs");
      std_msgs::Header optical_head = std_msgs::Header();
      optical_head.stamp = head.stamp;
      optical_head.frame_id = this->optical_frame_id_;

      // currently the unit vector calculation seems to be missing in the ifm3d state: therefore we don't publish anything to the uvec pubisher
      // publish unit vectors once on a latched topic, then re-initialize the
      // framegrabber with the user's requested schema mask
      if (! got_uvec)
        {
          lock.lock();
          sensor_msgs::ImagePtr uvec_msg =
            cv_bridge::CvImage(optical_head,
                               enc::TYPE_32FC3,
                               this->im_->UnitVectors()).toImageMsg();
          NODELET_INFO_STREAM("uvec image size: " << this->im_->UnitVectors().size());
          lock.unlock();
          this->uvec_pub_.publish(uvec_msg);
          got_uvec = true;
          ROS_INFO("Got unit vectors, restarting framegrabber with mask: %d",
                   (int) this->schema_mask_);

          while (! this->InitStructures(this->schema_mask_, this->pcic_port_))
            {
              ROS_WARN("Could not re-initialize pixel stream!");
              ros::Duration(1.0).sleep();
            }

          NODELET_INFO_STREAM("Start streaming data");
          continue;
        }

      //
      // Pull out all the wrapped images so that we can release the "GIL"
      // while publishing
      //
      lock.lock();

      NODELET_DEBUG_STREAM("start getting data");
      try
      {
        // boost::shared_ptr vs std::shared_ptr forces this copy
        pcl::copyPointCloud(*(this->im_->Cloud().get()), *cloud);
        xyz_img = this->im_->XYZImage();
        confidence_img = this->im_->ConfidenceImage();
        distance_img = this->im_->DistanceImage();
        amplitude_img = this->im_->AmplitudeImage();
        raw_amplitude_img = this->im_->RawAmplitudeImage();
        gray_img = this->im_->GrayImage();
        extrinsics = this->im_->Extrinsics();
        rgb_img = this->im_->JPEGImage();
      }
      catch (const ifm3d::error_t& ex)
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

      NODELET_DEBUG_STREAM("start publishing");
      // Confidence image is invariant - no need to check the mask
      sensor_msgs::ImagePtr confidence_msg =
        cv_bridge::CvImage(optical_head,
                           "mono8",
                           confidence_img).toImageMsg();
      this->conf_pub_.publish(confidence_msg);
      NODELET_DEBUG_STREAM("after publishing confidence image");

      if ((this->schema_mask_ & ifm3d::IMG_CART) == ifm3d::IMG_CART)
        {
          cloud->header = pcl_conversions::toPCL(head);
          this->cloud_pub_.publish(cloud);

          sensor_msgs::ImagePtr xyz_image_msg =
            cv_bridge::CvImage(head,
                               xyz_img.type() == CV_32FC3 ?
                               enc::TYPE_32FC3 : enc::TYPE_16SC3,
                               xyz_img).toImageMsg();
          this->xyz_image_pub_.publish(xyz_image_msg);
          NODELET_DEBUG_STREAM("after publishing xyz image");
        }

      if ((this->schema_mask_ & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
        {
          sensor_msgs::ImagePtr distance_msg =
            cv_bridge::CvImage(optical_head,
                               distance_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               distance_img).toImageMsg();
          this->distance_pub_.publish(distance_msg);
          NODELET_DEBUG_STREAM("after publishing distance image");
        }

      // this image is currently not available via the ifm3d
      // if ((this->schema_mask_ & ifm3d::IMG_DIS_NOISE) == ifm3d::IMG_DIS_NOISE)
      //   {
      //     sensor_msgs::ImagePtr distance_noise_msg =
      //       cv_bridge::CvImage(optical_head,
      //                          distance_noise_img.type() == CV_32FC1 ?
      //                          enc::TYPE_32FC1 : enc::TYPE_16UC1,
      //                          distance_noise_img).toImageMsg();
      //     this->distance_noise_pub_.publish(distance_noise_msg);
      //   }

      if ((this->schema_mask_ & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
        {
          sensor_msgs::ImagePtr amplitude_msg =
            cv_bridge::CvImage(optical_head,
                               amplitude_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               amplitude_img).toImageMsg();
          this->amplitude_pub_.publish(amplitude_msg);
          NODELET_DEBUG_STREAM("after publishing amplitude image");
        }

      if ((this->schema_mask_ & ifm3d::IMG_RAMP) == ifm3d::IMG_RAMP)
        {
          sensor_msgs::ImagePtr raw_amplitude_msg =
            cv_bridge::CvImage(optical_head,
                               raw_amplitude_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               raw_amplitude_img).toImageMsg();
          this->raw_amplitude_pub_.publish(raw_amplitude_msg);
          NODELET_DEBUG_STREAM("after publishing raw amplitude image");
        }

      if ((this->schema_mask_ & ifm3d::IMG_GRAY) == ifm3d::IMG_GRAY)
        {
          sensor_msgs::ImagePtr gray_image_msg =
            cv_bridge::CvImage(optical_head,
                               gray_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               gray_img).toImageMsg();
          this->gray_image_pub_.publish(gray_image_msg);
          NODELET_DEBUG_STREAM("after publishing gray image");
        }


      good_bad_pixels_img = cv::Mat::ones(confidence_img.rows,
                                          confidence_img.cols,
                                          CV_8UC1);

      // TODO: this casting of the confidence image to a boolean value image needs to be tested:
      // inv cast might be reqiured depending on the interpretation of the binary image
      int const thresh = 10;
      int const max_binary_value = 20;
      cv::threshold(confidence_img, good_bad_pixels_img, thresh, max_binary_value, cv::THRESH_BINARY);

      sensor_msgs::ImagePtr good_bad_msg =
        cv_bridge::CvImage(optical_head,
                           "mono8",
                           good_bad_pixels_img).toImageMsg();
      this->good_bad_pub_.publish(good_bad_msg);
      NODELET_DEBUG_STREAM("after publishing good/bad pixel image image");

      // The 2D is not yet settable in the schema mask: publish all the time

    if (!rgb_img.empty())
    {
      cv::Mat im_decode = cv::imdecode(rgb_img, cv::IMREAD_UNCHANGED);
      sensor_msgs::ImagePtr rgb_image_msg =
        cv_bridge::CvImage(optical_head,
                            "rgb8",
                            im_decode).toImageMsg();
      this->rgb_image_pub_.publish(rgb_image_msg);
      NODELET_DEBUG_STREAM("after publishing rgb image");
    }

      //
      // publish extrinsics
      //
      NODELET_DEBUG_STREAM("start publishing extrinsics");
      ifm3d::Extrinsics extrinsics_msg;
      extrinsics_msg.header = optical_head;
      try
        {
          extrinsics_msg.tx = extrinsics.at(0);
          extrinsics_msg.ty = extrinsics.at(1);
          extrinsics_msg.tz = extrinsics.at(2);
          extrinsics_msg.rot_x = extrinsics.at(3);
          extrinsics_msg.rot_y = extrinsics.at(4);
          extrinsics_msg.rot_z = extrinsics.at(5);
        }
      catch (const std::out_of_range& ex)
        {
          ROS_WARN("out-of-range error fetching extrinsics");
        }
      this->extrinsics_pub_.publish(extrinsics_msg);

    } // end: while (ros::ok()) { ... }
} // end: Run()


PLUGINLIB_EXPORT_CLASS(ifm3d_ros::CameraNodelet, nodelet::Nodelet)

/*
 * Copyright (C) 2018 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d/Config.h>
#include <ifm3d/Dump.h>
#include <ifm3d/Extrinsics.h>
#include <ifm3d/SoftOff.h>
#include <ifm3d/SoftOn.h>
#include <ifm3d/SyncClocks.h>
#include <ifm3d/Trigger.h>

namespace enc = sensor_msgs::image_encodings;

void
ifm3d_ros::CameraNodelet::onInit()
{
  std::string nn = this->getName();
  NODELET_INFO_STREAM("onInit(): " << nn);

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
  this->np_.param("xmlrpc_port", xmlrpc_port, (int) ifm3d::DEFAULT_XMLRPC_PORT);
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
  this->np_.param("sync_clocks", this->sync_clocks_, false);
  this->np_.param("frame_latency_thresh", this->frame_latency_thresh_, 60.0f);
  this->np_.param("frame_id_base", frame_id_base, frame_id_base);

  this->xmlrpc_port_ = static_cast<std::uint16_t>(xmlrpc_port);
  this->schema_mask_ = static_cast<std::uint16_t>(schema_mask);

  this->frame_id_ = frame_id_base + "_link";
  this->optical_frame_id_ = frame_id_base + "_optical_link";

  //-------------------
  // Published topics
  //-------------------
  this->cloud_pub_ =
    this->np_.advertise<pcl::PointCloud<ifm3d::PointT> >("cloud", 1);
  this->distance_pub_ = this->it_->advertise("distance", 1);
  this->amplitude_pub_ = this->it_->advertise("amplitude", 1);
  this->raw_amplitude_pub_ = this->it_->advertise("raw_amplitude", 1);
  this->conf_pub_ = this->it_->advertise("confidence", 1);
  this->good_bad_pub_ = this->it_->advertise("good_bad_pixels", 1);
  this->xyz_image_pub_ = this->it_->advertise("xyz_image", 1);

  // we latch the unit vectors
  this->uvec_pub_ =
    this->np_.advertise<sensor_msgs::Image>("unit_vectors", 1, true);

  this->extrinsics_pub_ =
    this->np_.advertise<ifm3d::Extrinsics>("extrinsics", 1);

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

  this->sync_clocks_srv_ =
    this->np_.advertiseService<ifm3d::SyncClocks::Request,
                               ifm3d::SyncClocks::Response>
    ("SyncClocks", std::bind(&CameraNodelet::SyncClocks, this,
                             std::placeholders::_1,
                             std::placeholders::_2));

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
      res.config = this->cam_->ToJSONStr();
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
      this->cam_->FromJSONStr(req.json);
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

  try
    {
      this->fg_->SWTrigger();
    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
    }

  return true;
}

bool
ifm3d_ros::CameraNodelet::SyncClocks(ifm3d::SyncClocks::Request& req,
                                     ifm3d::SyncClocks::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  NODELET_INFO_STREAM("Syncing camera clock to system...");
  try
    {
      this->cam_->SetCurrentTime(-1);
    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
      NODELET_WARN_STREAM(res.status << ": " << res.msg);
      return false;
    }

  return true;
}

bool
ifm3d_ros::CameraNodelet::SoftOff(ifm3d::SoftOff::Request& req,
                                  ifm3d::SoftOff::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  int active_application = 0;

  try
    {
      active_application = this->cam_->ActiveApplication();
      if (active_application > 0)
        {
          json dict =
            {
              {"Apps",
               {{{"Index", std::to_string(active_application)},
                 {"TriggerMode",
                  std::to_string(
                    static_cast<int>(ifm3d::Camera::trigger_mode::SW))}}}
              }
            };

          this->cam_->FromJSON(dict);

          this->assume_sw_triggered_ = true;
          this->timeout_millis_ = this->soft_off_timeout_millis_;
          this->timeout_tolerance_secs_ =
            this->soft_off_timeout_tolerance_secs_;
        }
    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
      return false;
    }

  return true;
}

bool
ifm3d_ros::CameraNodelet::SoftOn(ifm3d::SoftOn::Request& req,
                                 ifm3d::SoftOn::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  int active_application = 0;

  try
    {
      active_application = this->cam_->ActiveApplication();
      if (active_application > 0)
        {
          json dict =
            {
              {"Apps",
               {{{"Index", std::to_string(active_application)},
                 {"TriggerMode",
                  std::to_string(
                    static_cast<int>(ifm3d::Camera::trigger_mode::FREE_RUN))}}}
              }
            };

          this->cam_->FromJSON(dict);

          this->assume_sw_triggered_ = false;
          this->timeout_millis_ = this->soft_on_timeout_millis_;
          this->timeout_tolerance_secs_ =
            this->soft_on_timeout_tolerance_secs_;
        }
    }
  catch (const ifm3d::error_t& ex)
    {
      res.status = ex.code();
      res.msg = ex.what();
      return false;
    }

  return true;
}

bool
ifm3d_ros::CameraNodelet::InitStructures(std::uint16_t mask)
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
      this->cam_ = ifm3d::Camera::MakeShared(this->camera_ip_,
                                             this->xmlrpc_port_,
                                             this->password_);
      ros::Duration(1.0).sleep();

      NODELET_INFO_STREAM("Initializing framegrabber...");
      this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, mask);

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

bool
ifm3d_ros::CameraNodelet::AcquireFrame()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  bool retval = false;

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

  //
  // Sync camera clock with system clock if necessary
  //
  if (this->sync_clocks_)
    {
      NODELET_INFO_STREAM("Syncing camera clock to system...");
      try
        {
          this->cam_ = ifm3d::Camera::MakeShared(this->camera_ip_,
                                                 this->xmlrpc_port_,
                                                 this->password_);
          this->cam_->SetCurrentTime(-1);
        }
      catch (const ifm3d::error_t& ex)
        {
          NODELET_WARN_STREAM("Failed to sync clocks!");
          NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
        }
    }
  else
    {
      NODELET_INFO_STREAM("Camera clock will not be sync'd to system clock");
    }

  //
  // We need to account for the case of when the nodelet is being started prior
  // to the camera being plugged in.
  //
  while (ros::ok() && (! this->InitStructures(ifm3d::IMG_UVEC)))
    {
      NODELET_WARN_STREAM("Could not initialize pixel stream!");
      ros::Duration(1.0).sleep();
    }

  pcl::PointCloud<ifm3d::PointT>::Ptr
    cloud(new pcl::PointCloud<ifm3d::PointT>());

  cv::Mat confidence_img;
  cv::Mat distance_img;
  cv::Mat amplitude_img;
  cv::Mat xyz_img;
  cv::Mat raw_amplitude_img;
  cv::Mat good_bad_pixels_img;

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
                                            : ifm3d::IMG_UVEC))
                {
                  NODELET_WARN_STREAM("Could not re-initialize pixel stream!");
                  ros::Duration(1.0).sleep();
                }

              last_frame = ros::Time::now();
            }

          continue;
        }

      last_frame = ros::Time::now();

      std_msgs::Header head = std_msgs::Header();
      head.frame_id = this->frame_id_;
      head.stamp = ros::Time(
        std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>
        (this->im_->TimeStamp().time_since_epoch()).count());
      if ((ros::Time::now() - head.stamp) >
          ros::Duration(this->frame_latency_thresh_))
        {
          ROS_WARN_ONCE("Camera's time is not up to date, therefore header's "
            "timestamps will be the reception time and not capture time. "
            "Please update the camera's time if you need the capture time.");
          head.stamp = ros::Time::now();
        }

      std_msgs::Header optical_head = std_msgs::Header();
      optical_head.stamp = head.stamp;
      optical_head.frame_id = this->optical_frame_id_;

      // publish unit vectors once on a latched topic, then re-initialize the
      // framegrabber with the user's requested schema mask
      if (! got_uvec)
        {
          lock.lock();
          sensor_msgs::ImagePtr uvec_msg =
            cv_bridge::CvImage(optical_head,
                               enc::TYPE_32FC3,
                               this->im_->UnitVectors()).toImageMsg();
          lock.unlock();
          this->uvec_pub_.publish(uvec_msg);
          got_uvec = true;
          ROS_INFO("Got unit vectors, restarting framegrabber with mask: %d",
                   (int) this->schema_mask_);

          while (! this->InitStructures(this->schema_mask_))
            {
              ROS_WARN("Could not re-initialize pixel stream!");
              ros::Duration(1.0).sleep();
            }

          // should solve the problem of first image being (0,0)
          // see: https://github.com/lovepark/ifm3d/issues/12
          continue;
        }

      //
      // Pull out all the wrapped images so that we can release the "GIL"
      // while publishing
      //
      lock.lock();

      // boost::shared_ptr vs std::shared_ptr forces this copy
      pcl::copyPointCloud(*(this->im_->Cloud().get()), *cloud);
      xyz_img = this->im_->XYZImage();
      confidence_img = this->im_->ConfidenceImage();
      distance_img = this->im_->DistanceImage();
      amplitude_img = this->im_->AmplitudeImage();
      raw_amplitude_img = this->im_->RawAmplitudeImage();
      extrinsics = this->im_->Extrinsics();

      lock.unlock();

      //
      // Now, do the publishing
      //

      // Confidence image is invariant - no need to check the mask
      sensor_msgs::ImagePtr confidence_msg =
        cv_bridge::CvImage(optical_head,
                           "mono8",
                           confidence_img).toImageMsg();
      this->conf_pub_.publish(confidence_msg);

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
        }

      if ((this->schema_mask_ & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
        {
          sensor_msgs::ImagePtr distance_msg =
            cv_bridge::CvImage(optical_head,
                               distance_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               distance_img).toImageMsg();
          this->distance_pub_.publish(distance_msg);
        }

      if ((this->schema_mask_ & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
        {
          sensor_msgs::ImagePtr amplitude_msg =
            cv_bridge::CvImage(optical_head,
                               amplitude_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               amplitude_img).toImageMsg();
          this->amplitude_pub_.publish(amplitude_msg);
        }

      if ((this->schema_mask_ & ifm3d::IMG_RAMP) == ifm3d::IMG_RAMP)
        {
          sensor_msgs::ImagePtr raw_amplitude_msg =
            cv_bridge::CvImage(optical_head,
                               raw_amplitude_img.type() == CV_32FC1 ?
                               enc::TYPE_32FC1 : enc::TYPE_16UC1,
                               raw_amplitude_img).toImageMsg();
          this->raw_amplitude_pub_.publish(raw_amplitude_msg);
        }

      //
      // XXX: Need to publish ambient light / gray image
      // ... however, as of now (3/26/2017) there is no
      // imager mode on the O3X that actually supports it
      //
      // Update: as of 5/10/2018 still not supported.
      //

      good_bad_pixels_img = cv::Mat::ones(confidence_img.rows,
                                          confidence_img.cols,
                                          CV_8UC1);
      cv::bitwise_and(confidence_img, good_bad_pixels_img,
                      good_bad_pixels_img);
      sensor_msgs::ImagePtr good_bad_msg =
        cv_bridge::CvImage(optical_head,
                           "mono8",
                           (good_bad_pixels_img == 0)).toImageMsg();
      this->good_bad_pub_.publish(good_bad_msg);

      //
      // publish extrinsics
      //
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

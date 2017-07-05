/*
 * Copyright (C) 2017 Love Park Robotics, LLC
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
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d/Config.h>
#include <ifm3d/Dump.h>
#include <ifm3d/Trigger.h>

namespace enc = sensor_msgs::image_encodings;

class IFM3DNode
{
public:
  IFM3DNode()
    : schema_mask_(ifm3d::DEFAULT_SCHEMA_MASK),
      timeout_millis_(500),
      timeout_tolerance_secs_(5.0),
      assume_sw_triggered_(false),
      spinner_(new ros::AsyncSpinner(1))
  {
    int schema_mask;
    std::string frame_id_base;

    ros::NodeHandle nh; // public
    ros::NodeHandle np("~"); // private

    np.param("ip", this->camera_ip_, ifm3d::DEFAULT_IP);
    np.param("xmlrpc_port", this->xmlrpc_port_,
             (int) ifm3d::DEFAULT_XMLRPC_PORT);
    np.param("password", this->password_, ifm3d::DEFAULT_PASSWORD);
    np.param("schema_mask", schema_mask, (int) ifm3d::DEFAULT_SCHEMA_MASK);
    np.param("timeout_millis", this->timeout_millis_, 500);
    np.param("timeout_tolerance_secs", this->timeout_tolerance_secs_, 5.0);
    np.param("assume_sw_triggered", this->assume_sw_triggered_, false);
    np.param("frame_id_base", frame_id_base,
             std::string(ros::this_node::getName()).substr(1));

    this->schema_mask_ = static_cast<std::uint16_t>(schema_mask);

    this->frame_id_ = frame_id_base + "_link";
    this->optical_frame_id_ = frame_id_base + "_optical_link";

    //----------------------
    // Published topics
    //----------------------
    this->cloud_pub_ =
      nh.advertise<pcl::PointCloud<ifm3d::PointT> >("cloud", 1);

    image_transport::ImageTransport it(nh);
    this->distance_pub_ = it.advertise("distance", 1);
    this->amplitude_pub_ = it.advertise("amplitude", 1);
    this->raw_amplitude_pub_ = it.advertise("raw_amplitude", 1);
    this->conf_pub_ = it.advertise("confidence", 1);
    this->good_bad_pub_ = it.advertise("good_bad_pixels", 1);
    this->xyz_image_pub_ = it.advertise("xyz_image", 1);

    // NOTE: not using ImageTransport here ... having issues with the
    // latching. I need to investigate further. A "normal" publisher seems to
    // work.
    this->uvec_pub_ =
      nh.advertise<sensor_msgs::Image>("unit_vectors", 1, true);

    //this->extrinsics_pub_ = nh.advertise<ifm3d::Extrinsics>("extrinsics", 1);

    //---------------------
    // Advertised Services
    //---------------------
    this->dump_srv_ =
      nh.advertiseService<ifm3d::Dump::Request, ifm3d::Dump::Response>
      ("Dump", std::bind(&IFM3DNode::Dump, this,
                         std::placeholders::_1,
                         std::placeholders::_2));

    this->config_srv_ =
      nh.advertiseService<ifm3d::Config::Request, ifm3d::Config::Response>
      ("Config", std::bind(&IFM3DNode::Config, this,
                           std::placeholders::_1,
                           std::placeholders::_2));

    this->trigger_srv_ =
      nh.advertiseService<ifm3d::Trigger::Request, ifm3d::Trigger::Response>
      ("Trigger", std::bind(&IFM3DNode::Trigger, this,
                            std::placeholders::_1,
                            std::placeholders::_2));

  } // end: ctor

  /**
   * Publishing loop
   */
  void Run()
  {
    std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);
    this->spinner_->start();

    // atomically re-initializes the core data structures for interfacing
    // to the camera
    auto init_structures =
      [this](std::uint16_t mask)->bool
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        bool retval = false;

        try
          {
            ROS_INFO("Running dtors...");
            this->im_.reset();
            this->fg_.reset();
            this->cam_.reset();

            ROS_INFO("Initializing camera...");
            this->cam_ =
              ifm3d::Camera::MakeShared(this->camera_ip_,
                                        this->xmlrpc_port_,
                                        this->password_);
            ros::Duration(1.0).sleep();

            ROS_INFO("Initializing framegrabber...");
            this->fg_ =
              std::make_shared<ifm3d::FrameGrabber>(this->cam_, mask);

            ROS_INFO("Initializing image buffer...");
            this->im_ = std::make_shared<ifm3d::ImageBuffer>();

            retval = true;
          }
        catch (const ifm3d::error_t& ex)
          {
            ROS_WARN_STREAM(ex.code() << ": " << ex.what());
            this->im_.reset();
            this->fg_.reset();
            this->cam_.reset();
            retval = false;
          }

        return retval;
      };

    // simplifies "safe" image acquisition
    auto acquire_frame =
      [this]()->bool
      {
        std::lock_guard<std::mutex> lock(this->mutex_);
        bool retval = false;

        try
          {
            retval =
              this->fg_->WaitForFrame(this->im_.get(), this->timeout_millis_);
          }
        catch (const ifm3d::error_t& ex)
          {
            ROS_WARN_STREAM(ex.code() << ": " << ex.what());
            retval = false;
          }

        return retval;
      };

    // This accounts for the case where the node is started
    // prior to the camera being plugged in
    while (! init_structures(ifm3d::IMG_UVEC))
      {
        ROS_WARN("Could not initialize pixel stream!");
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

    ros::Time last_frame = ros::Time::now();
    bool got_uvec = false;

    while (ros::ok())
      {
        if (! acquire_frame())
          {
            if (! this->assume_sw_triggered_)
              {
                ROS_WARN("Timeout waiting for camera!");
              }
            else
              {
                ros::Duration(.001).sleep();
              }

            if ((ros::Time::now() - last_frame).toSec() >
                this->timeout_tolerance_secs_)
              {
                ROS_WARN("Attempting to restart framegrabber...");
                while (! init_structures(got_uvec ?
                                         this->schema_mask_ : ifm3d::IMG_UVEC))
                  {
                    ROS_WARN("Could not re-initialize pixel stream!");
                    ros::Duration(1.0).sleep();
                  }

                last_frame = ros::Time::now();
              }
            continue;
          }

        std_msgs::Header head = std_msgs::Header();
        head.stamp = ros::Time::now();
        head.frame_id = this->frame_id_;
        last_frame = head.stamp;

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

            while (! init_structures(this->schema_mask_))
                  {
                    ROS_WARN("Could not re-initialize pixel stream!");
                    ros::Duration(1.0).sleep();
                  }
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
        // XXX: Need to publish extrinsics
        //

      } // end: while(ros::ok()) {...}
  } // end: Run()

  /**
   * Implements the `Dump' service
   *
   * The `Dump' service will dump the current camera configuration to a JSON
   * string. This JSON string is suitable for editing and using to reconfigure
   * the camera via the `Config' service.
   */
  bool Dump(ifm3d::Dump::Request &req,
            ifm3d::Dump::Response &res)
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
      }

    return true;
  }

  /**
   * Implements the `Config' service.
   *
   * The `Config' service will read the input JSON configuration data and
   * mutate the camera's settings to match that of the configuration
   * described by the JSON file. Syntactically, the JSON should look like the
   * JSON that is produced by `Dump'. However, you need not specify every
   * parameter. You can specify only the parameters you wish to change with the
   * only caveat being that you need to specify the parameter as fully
   * qualified from the top-level root of the JSON tree.
   */
  bool Config(ifm3d::Config::Request &req,
              ifm3d::Config::Response &res)
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

    return true;
  }

  /**
   * Implements the `Trigger' service.
   *
   * For cameras whose active application is set to software triggering as
   * opposed to free-running, this service send the trigger for image
   * acquisition to the camera.
   */
  bool Trigger(ifm3d::Trigger::Request &req,
               ifm3d::Trigger::Response &res)
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

private:
  std::string camera_ip_;
  int xmlrpc_port_;
  std::string password_;
  std::uint16_t schema_mask_;
  int timeout_millis_;
  double timeout_tolerance_secs_;
  bool assume_sw_triggered_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ifm3d::Camera::Ptr cam_;
  ifm3d::FrameGrabber::Ptr fg_;
  ifm3d::ImageBuffer::Ptr im_;
  std::mutex mutex_;

  std::string frame_id_;
  std::string optical_frame_id_;

  ros::Publisher cloud_pub_;
  ros::Publisher uvec_pub_;
  ros::Publisher extrinsics_pub_;
  image_transport::Publisher distance_pub_;
  image_transport::Publisher amplitude_pub_;
  image_transport::Publisher raw_amplitude_pub_;
  image_transport::Publisher conf_pub_;
  image_transport::Publisher good_bad_pub_;
  image_transport::Publisher xyz_image_pub_;

  ros::ServiceServer dump_srv_;
  ros::ServiceServer config_srv_;
  ros::ServiceServer trigger_srv_;

}; // end: class IFM3DNode

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ifm3d");
  IFM3DNode().Run();
  return 0;
}

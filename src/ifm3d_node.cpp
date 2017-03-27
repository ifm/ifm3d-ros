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

    //-----------------------------------------
    // Instantiate the camera and frame-grabber
    //-----------------------------------------
    this->cam_ =
      ifm3d::Camera::MakeShared(this->camera_ip_,
                                this->xmlrpc_port_,
                                this->password_);

    // NOTE: we initially only want to stream in the unit vectors, we switch
    // to the requested mask, *after* we publish the unit vectors at least
    // once.
    this->fg_ =
      std::make_shared<ifm3d::FrameGrabber>(this->cam_, ifm3d::IMG_UVEC);

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
  } // end: ctor

  /**
   * Publishing loop
   */
  void Run()
  {
    std::unique_lock<std::mutex> cam_lock(this->cam_mutex_, std::defer_lock);
    std::unique_lock<std::mutex> fg_lock(this->fg_mutex_, std::defer_lock);
    this->spinner_->start();

    auto buff = std::make_shared<ifm3d::ImageBuffer>();

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
        fg_lock.lock();
        if (! this->fg_->WaitForFrame(buff.get(), this->timeout_millis_))
          {
            fg_lock.unlock();
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
                // This is different than in o3d3xx-ros b/c
                // of the constraint on O3X where it can handle
                // only a single PCIC client socket
                cam_lock.lock();
                fg_lock.lock();

                buff.reset();
                this->fg_.reset();
                this->cam_.reset();

                this->cam_ =
                  ifm3d::Camera::MakeShared(this->camera_ip_,
                                            this->xmlrpc_port_,
                                            this->password_);
                if (got_uvec)
                  {
                    this->fg_ =
                      std::make_shared<ifm3d::FrameGrabber>(this->cam_,
                                                            this->schema_mask_);
                  }
                else
                  {
                    this->fg_ =
                      std::make_shared<ifm3d::FrameGrabber>(this->cam_,
                                                            ifm3d::IMG_UVEC);
                  }

                buff = std::make_shared<ifm3d::ImageBuffer>();

                fg_lock.unlock();
                cam_lock.unlock();

                last_frame = ros::Time::now();
              }
            continue;
          }
        fg_lock.unlock();

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
            sensor_msgs::ImagePtr uvec_msg =
              cv_bridge::CvImage(optical_head,
                                 enc::TYPE_32FC3,
                                 buff->UnitVectors()).toImageMsg();
            this->uvec_pub_.publish(uvec_msg);

            ROS_INFO("Got unit vectors, restarting framegrabber with mask: %d",
                     (int) this->schema_mask_);

            cam_lock.lock();
            fg_lock.lock();

            got_uvec = true;

            buff.reset();
            buff = std::make_shared<ifm3d::ImageBuffer>();

            this->fg_.reset();
            this->fg_ =
              std::make_shared<ifm3d::FrameGrabber>(this->cam_,
                                                    this->schema_mask_);

            fg_lock.unlock();
            cam_lock.unlock();

            continue;
          }

        // Confidence image is invariant - no need to check the mask
        confidence_img = buff->ConfidenceImage();
        sensor_msgs::ImagePtr confidence_msg =
          cv_bridge::CvImage(optical_head,
                             "mono8",
                             confidence_img).toImageMsg();
        this->conf_pub_.publish(confidence_msg);

        if ((this->schema_mask_ & ifm3d::IMG_CART) == ifm3d::IMG_CART)
          {
            // boost::shared_ptr vs std::shared_ptr forces this copy
            pcl::copyPointCloud(*(buff->Cloud().get()), *cloud);
            cloud->header = pcl_conversions::toPCL(head);
            this->cloud_pub_.publish(cloud);

            xyz_img = buff->XYZImage();
            sensor_msgs::ImagePtr xyz_image_msg =
              cv_bridge::CvImage(head,
                                 xyz_img.type() == CV_32FC3 ?
                                 enc::TYPE_32FC3 : enc::TYPE_16SC3,
                                 xyz_img).toImageMsg();
            this->xyz_image_pub_.publish(xyz_image_msg);
          }

        if ((this->schema_mask_ & ifm3d::IMG_RDIS) == ifm3d::IMG_RDIS)
          {
            distance_img = buff->DistanceImage();
            sensor_msgs::ImagePtr distance_msg =
              cv_bridge::CvImage(optical_head,
                                 distance_img.type() == CV_32FC1 ?
                                 enc::TYPE_32FC1 : enc::TYPE_16UC1,
                                 distance_img).toImageMsg();
            this->distance_pub_.publish(distance_msg);
          }

        if ((this->schema_mask_ & ifm3d::IMG_AMP) == ifm3d::IMG_AMP)
          {
            amplitude_img = buff->AmplitudeImage();
            sensor_msgs::ImagePtr amplitude_msg =
              cv_bridge::CvImage(optical_head,
                                 amplitude_img.type() == CV_32FC1 ?
                                 enc::TYPE_32FC1 : enc::TYPE_16UC1,
                                 amplitude_img).toImageMsg();
            this->amplitude_pub_.publish(amplitude_msg);
          }

        if ((this->schema_mask_ & ifm3d::IMG_RAMP) == ifm3d::IMG_RAMP)
          {
            raw_amplitude_img = buff->RawAmplitudeImage();
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
  std::mutex cam_mutex_;
  ifm3d::FrameGrabber::Ptr fg_;
  std::mutex fg_mutex_;

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

}; // end: class IFM3DNode

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ifm3d");
  IFM3DNode().Run();
  return 0;
}

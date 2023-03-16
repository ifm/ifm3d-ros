// -*- c++ -*-

// SPDX-License-Identifier: Apache-2.0
//  Copyright (C) 2021 ifm electronic, gmbh

#ifndef __IFM3D_ROS_CAMERA_NODELET_H__
#define __IFM3D_ROS_CAMERA_NODELET_H__

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ifm3d/device/device.h>
#include <ifm3d/fg.h>
#include <ifm3d/fg/frame_grabber.h>
#include <ifm3d_ros_msgs/Config.h>
#include <ifm3d_ros_msgs/Dump.h>
#include <ifm3d_ros_msgs/Extrinsics.h>
#include <ifm3d_ros_msgs/SoftOff.h>
#include <ifm3d_ros_msgs/SoftOn.h>
#include <ifm3d_ros_msgs/Trigger.h>

namespace ifm3d_legacy
{
  const std::uint16_t IMG_RDIS = (1 << 0);        // 2**0
  const std::uint16_t IMG_AMP = (1 << 1);         // 2**1
  const std::uint16_t IMG_RAMP = (1 << 2);        // 2**2
  const std::uint16_t IMG_CART = (1 << 3);        // 2**3
//  const std::uint16_t IMG_UVEC = (1 << 4);        // 2**4
//  const std::uint16_t EXP_TIME = (1 << 5);        // 2**5
//  const std::uint16_t IMG_GRAY = (1 << 6);        // 2**6
//  const std::uint16_t ILLU_TEMP = (1 << 7);       // 2**7
//  const std::uint16_t INTR_CAL = (1 << 8);        // 2**8
//  const std::uint16_t INV_INTR_CAL = (1 << 9);    // 2**9
//  const std::uint16_t JSON_MODEL = (1 << 10);     // 2**10
//  const std::uint16_t IMG_DIS_NOISE = (1 << 11);  // 2**11

  std::map<std::uint16_t, ifm3d::buffer_id> schema_mask_buffer_id_map{
    {IMG_RDIS, ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE},
    {IMG_AMP, ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE},
    {IMG_RAMP, ifm3d::buffer_id::AMPLITUDE_IMAGE},
    {IMG_CART, ifm3d::buffer_id::XYZ}
  };

  ifm3d::FrameGrabber::BufferList buffer_list_from_schema_mask(const std::uint16_t mask)
  {
    ifm3d::FrameGrabber::BufferList buffer_list;

    for (auto& [schema_mask, buffer_id]: schema_mask_buffer_id_map) {
      if ((mask & schema_mask) == schema_mask)
      {
        buffer_list.emplace_back(buffer_id);
      }
    }
    return buffer_list;
  }
}  // namespace ifm3d_legacy

namespace ifm3d_ros
{
/**
 * This class implements the ROS nodelet interface to allow for running
 * in-process data transport between ifm3d image data and ROS consumers. This
 * class is used to manage, configure, and acquire data from a single ifm3d
 * camera.
 */
class CameraNodelet : public nodelet::Nodelet
{
private:
  //
  // Nodelet lifecycle functions
  //
  void onInit() override;

  //
  // ROS services
  //
  bool Dump(ifm3d_ros_msgs::Dump::Request& req, ifm3d_ros_msgs::Dump::Response& res);
  bool Config(ifm3d_ros_msgs::Config::Request& req, ifm3d_ros_msgs::Config::Response& res);
  bool Trigger(ifm3d_ros_msgs::Trigger::Request& req, ifm3d_ros_msgs::Trigger::Response& res);
  bool SoftOff(ifm3d_ros_msgs::SoftOff::Request& req, ifm3d_ros_msgs::SoftOff::Response& res);
  bool SoftOn(ifm3d_ros_msgs::SoftOn::Request& req, ifm3d_ros_msgs::SoftOn::Response& res);

  //
  // This is our main publishing loop and its helper functions
  //
  void Run();
  bool InitStructures(std::uint16_t pcic_port);
  void Callback(ifm3d::Frame::Ptr frame);
  bool StartStream();

  //
  // state
  //
  std::string camera_ip_;
  std::uint16_t xmlrpc_port_;
  std::uint16_t pcic_port_;
  std::string password_;
  ifm3d::FrameGrabber::BufferList schema_mask_default_;
  std::uint16_t schema_mask_;
  ifm3d::TimePointT last_frame_time_;

  int timeout_millis_;
  double timeout_tolerance_secs_;
  bool assume_sw_triggered_;
  int soft_on_timeout_millis_;
  double soft_on_timeout_tolerance_secs_;
  int soft_off_timeout_millis_;
  double soft_off_timeout_tolerance_secs_;
  float frame_latency_thresh_;

  std::string frame_id_;
  std::string optical_frame_id_;

  ifm3d::Device::Ptr cam_;
  ifm3d::FrameGrabber::Ptr fg_;
  std::mutex mutex_;
  bool receiving_;

  ros::NodeHandle np_;
  std::unique_ptr<image_transport::ImageTransport> it_;

  //
  // Topics we publish
  //
  ros::Publisher cloud_pub_;
  ros::Publisher uvec_pub_;
  ros::Publisher extrinsics_pub_;
  image_transport::Publisher distance_pub_;
  image_transport::Publisher distance_noise_pub_;
  image_transport::Publisher amplitude_pub_;
  image_transport::Publisher raw_amplitude_pub_;
  image_transport::Publisher conf_pub_;
  image_transport::Publisher gray_image_pub_;
  ros::Publisher rgb_image_pub_;

  //
  // Services we advertise
  //
  ros::ServiceServer dump_srv_;
  ros::ServiceServer config_srv_;
  ros::ServiceServer trigger_srv_;
  ros::ServiceServer soft_off_srv_;
  ros::ServiceServer soft_on_srv_;

  //
  // We use a ROS one-shot timer to kick off our publishing loop.
  //
  ros::Timer publoop_timer_;

  //
  // Message header used for publishing
  //
  std_msgs::Header optical_head;
  std_msgs::Header head;


};  // end: class CameraNodelet

}  // namespace ifm3d_ros

#endif  // __IFM3D_ROS_CAMERA_NODELET_H__

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
  void Callback2D(ifm3d::Frame::Ptr frame);
  void Callback3D(ifm3d::Frame::Ptr frame);
  bool StartStream();

  //
  // state
  //
  std::string camera_ip_;
  std::uint16_t xmlrpc_port_;
  std::uint16_t pcic_port_;
  std::string password_;
  std::string imager_type_;
  ifm3d::TimePointT last_frame_time_;

  bool xyz_image_stream_;
  bool confidence_image_stream_;
  bool radial_distance_image_stream_;
  bool radial_distance_noise_stream_;
  bool amplitude_image_stream_;
  bool extrinsic_image_stream_;
  bool intrinsic_image_stream_;
  bool rgb_image_stream_;

  std::list<ifm3d::buffer_id> buffer_list;
  ifm3d::FrameGrabber::BufferList schema_mask_default_3d_;
  ifm3d::FrameGrabber::BufferList schema_mask_default_2d_;

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
  ros::Publisher intrinsics_pub_;
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

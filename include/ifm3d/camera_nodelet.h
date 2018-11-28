// -*- c++ -*-
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

#ifndef __IFM3D_ROS_CAMERA_NODELET_H__
#define __IFM3D_ROS_CAMERA_NODELET_H__

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

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
    virtual void onInit() override;

    //
    // ROS services
    //
    bool Dump(ifm3d::Dump::Request& req, ifm3d::Dump::Response& res);
    bool Config(ifm3d::Config::Request& req, ifm3d::Config::Response& res);
    bool Trigger(ifm3d::Trigger::Request& req, ifm3d::Trigger::Response& res);
    bool SoftOff(ifm3d::SoftOff::Request& req, ifm3d::SoftOff::Response& res);
    bool SoftOn(ifm3d::SoftOn::Request& req, ifm3d::SoftOn::Response& res);
    bool SyncClocks(ifm3d::SyncClocks::Request& req,
                    ifm3d::SyncClocks::Response& res);

    //
    // This is our main publishing loop and its helper functions
    //
    void Run();
    bool InitStructures(std::uint16_t mask);
    bool AcquireFrame();

    //
    // state
    //
    std::string camera_ip_;
    std::uint16_t xmlrpc_port_;
    std::string password_;
    std::uint16_t schema_mask_;
    int timeout_millis_;
    double timeout_tolerance_secs_;
    bool assume_sw_triggered_;
    int soft_on_timeout_millis_;
    double soft_on_timeout_tolerance_secs_;
    int soft_off_timeout_millis_;
    double soft_off_timeout_tolerance_secs_;
    float frame_latency_thresh_;
    bool sync_clocks_;

    std::string frame_id_;
    std::string optical_frame_id_;

    ifm3d::Camera::Ptr cam_;
    ifm3d::FrameGrabber::Ptr fg_;
    ifm3d::ImageBuffer::Ptr im_;
    std::mutex mutex_;

    ros::NodeHandle np_;
    std::unique_ptr<image_transport::ImageTransport> it_;

    //
    // Topics we publish
    //
    ros::Publisher cloud_pub_;
    ros::Publisher uvec_pub_;
    ros::Publisher extrinsics_pub_;
    image_transport::Publisher distance_pub_;
    image_transport::Publisher amplitude_pub_;
    image_transport::Publisher raw_amplitude_pub_;
    image_transport::Publisher conf_pub_;
    image_transport::Publisher good_bad_pub_;
    image_transport::Publisher xyz_image_pub_;

    //
    // Services we advertise
    //
    ros::ServiceServer dump_srv_;
    ros::ServiceServer config_srv_;
    ros::ServiceServer trigger_srv_;
    ros::ServiceServer soft_off_srv_;
    ros::ServiceServer soft_on_srv_;
    ros::ServiceServer sync_clocks_srv_;

    //
    // We use a ROS one-shot timer to kick off our publishing loop.
    //
    ros::Timer publoop_timer_;

  }; // end: class CameraNodelet

} // end: namespace ifm3d_ros

#endif // __IFM3D_ROS_CAMERA_NODELET_H__

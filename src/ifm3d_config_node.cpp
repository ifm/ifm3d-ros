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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include "ifm3d/Config.h"

int main(int argc, char **argv)
{
  std::string infile;
  std::string json;

  ros::init(argc, argv, "ifm3d_config");

  ros::NodeHandle nh("~");
  nh.param("infile", infile, std::string("-"));

  if (infile == "-")
  {
    std::string line;
    std::ostringstream buff;
    while (std::getline(std::cin, line))
    {
      buff << line << std::endl;
    }

    json.assign(buff.str());
  }
  else
  {
    std::ifstream ifs(infile, std::ios::in);
    if (! ifs)
    {
      ROS_ERROR("Failed to open file: %s", infile.c_str());
      return -1;
    }

    json.assign((std::istreambuf_iterator<char>(ifs)),
                (std::istreambuf_iterator<char>()));
  }

  ros::ServiceClient client = nh.serviceClient<ifm3d::Config>("/Config");

  ifm3d::Config srv;
  srv.request.json = json;

  if (client.call(srv))
  {
    ROS_INFO("status=%d", srv.response.status);
    ROS_INFO("msg=%s", srv.response.msg.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call `Config' service!");
    return 1;
  }

  return 0;
}

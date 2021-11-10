# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2021 ifm electronic, gmbh

"""
Mimics (mostly) the `ifm3d config` command but communicates through the ROS
graph
"""

import json
import rospy
import sys
from ifm3d_ros_msgs.srv import Config

SRV_TIMEOUT = 2.0 # seconds
SRV_NAME = "/ifm3d_ros_examples/camera/Config"

class ConfigClient(object):

    def __init__(self):
        rospy.init_node('ifm3d_config_client')

    def run(self):
        j = json.load(sys.stdin)
        rospy.wait_for_service(SRV_NAME, timeout=SRV_TIMEOUT)
        proxy = rospy.ServiceProxy(SRV_NAME, Config)
        resp = proxy(json.dumps(j))
        if resp.status != 0:
            print("Error: %s - %s" % (str(resp.status), resp.msg))

        return resp.status

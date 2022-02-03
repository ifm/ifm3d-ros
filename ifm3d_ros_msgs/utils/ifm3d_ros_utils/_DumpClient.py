# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2021 ifm electronic, gmbh

"""
Mimics the `ifm3d dump` command but communicates through the ROS graph
"""

import json
import rospy
import sys
from ifm3d_ros_msgs.srv import Dump

SRV_TIMEOUT = 2.0 # seconds
SRV_NAME = "/ifm3d_ros_examples/camera/Dump"

class DumpClient(object):

    def __init__(self):
        rospy.init_node('ifm3d_dump_client')

    def run(self):
        rospy.wait_for_service(SRV_NAME, timeout=SRV_TIMEOUT)
        proxy = rospy.ServiceProxy(SRV_NAME, Dump)
        resp = proxy()
        if resp.status == 0:
            print(json.dumps(json.loads(resp.config),
                             sort_keys=True, indent=4, separators=(',', ': ')))
        else:
            sys.stderr.write("Dump failed with error: %s - %s\n" %
                             (str(resp.status),
                              "Check the `ifm3d' logs for  more detail"))

        return resp.status

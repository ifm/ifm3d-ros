#
# Copyright (C)  2018 ifm electronic, gmbh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distribted on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""
Mimics (mostly) the `ifm3d config` command but communicates through the ROS
graph
"""

import json
import rospy
import sys
from ifm3d.srv import Config

SRV_TIMEOUT = 2.0 # seconds
SRV_NAME = "/ifm3d/camera/Config"

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

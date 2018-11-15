#!/usr/bin/env python
# -*- python -*-

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

import sys
import time
import unittest
import rospy
import rostest
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from ifm3d.msg import Extrinsics

# XXX: TP debugging
#np.set_printoptions(threshold=np.nan)

class TestCamera(unittest.TestCase):
    """
    TP: 14 May 2018 - heavily adapted from my old o3d3xx-ros test of the same
    name.

    This class tests the following:
      - Getting data from the camera
      - Computing the Cartesian data and comparing it to ground truth
      - To do the comparison to ground truth, the computed cartesian data
        must be transformed, to do that we use the tf2 API. It follows that
        this indirectly is testing our tf2 static publisher which is publishing
        the transform between the optical and camera frames

    NOTE: The camera h/w is needed to run this test.
    """

    def __init__(self, *args):
        super(TestCamera, self).__init__(*args)
        self.success_ = False

        self.cloud_ = None # ground truth for cartesian computation
        self.rdis_ = None
        self.uvec_ = None
        self.extrinsics_ = None
        self.conf_ = None

    def test_camera(self):
        time.sleep(5.0) # <-- wait for rosmaster and ifm3d nodelet
        rospy.init_node('test_camera')

        self.bridge_ = CvBridge()
        self.rdis_sub_ = \
          rospy.Subscriber("/ifm3d/camera/distance", Image, self.image_cb,
                           queue_size=None, callback_args="rdis")
        self.cloud_sub_ = \
          rospy.Subscriber("/ifm3d/camera/xyz_image", Image, self.image_cb,
                           queue_size=None, callback_args="cloud")
        self.uvec_sub_ = \
          rospy.Subscriber("/ifm3d/camera/unit_vectors", Image, self.image_cb,
                           queue_size=None, callback_args="uvec")
        self.extrinsics_sub_ = \
          rospy.Subscriber("/ifm3d/camera/extrinsics", Extrinsics,
                           self.extrinsics_cb, queue_size=None)
        self.conf_sub_ = \
          rospy.Subscriber("/ifm3d/camera/confidence", Image, self.image_cb,
                           queue_size=None, callback_args="conf")

        # If it takes more than 10 secs to run our test, something is wrong.
        timeout_t = time.time() + 10.0
        rate = rospy.Rate(10.0)
        while ((not rospy.is_shutdown()) and
               (not self.success_) and
               (time.time() < timeout_t)):
            # Make sure we have all the data we need to compute
            if ((self.rdis_ is not None) and
                (self.cloud_ is not None) and
                (self.uvec_ is not None) and
                (self.extrinsics_ is not None) and
                (self.conf_ is not None)):

                # Make sure the cloud, conf, and rdis are from the same image
                # acquisition.
                d = self.rdis_.header.stamp - self.cloud_.header.stamp
                if d.to_sec() == 0:
                    d2 = self.rdis_.header.stamp - self.conf_.header.stamp
                    if d2.to_sec() == 0:
                        self.success_ = self.compute_cartesian()
                        break
                    else:
                        # get new data
                        self.rdis_ = None
                        self.cloud_ = None
                        self.conf_ = None
                else:
                    # get new data
                    self.rdis_ = None
                    self.cloud_ = None
                    self.conf_ = None

            rate.sleep()

        self.assertTrue(self.success_)

    def extrinsics_cb(self, data, *args):
        if self.extrinsics_ is None:
            self.extrinsics_ = data

    def image_cb(self, data, *args):
        im_type = args[0]
        if im_type == "rdis":
            if self.rdis_ is None:
                self.rdis_ = data
        elif im_type == "cloud":
            if self.cloud_ is None:
                self.cloud_ = data
        elif im_type == "uvec":
            if self.uvec_ is None:
                self.uvec_ = data
        elif im_type == "conf":
            if self.conf_ is None:
                self.conf_ = data

    def compute_cartesian(self):
        """
        Computes the Cartesian data from the radial distance image, unit
        vectors, and extrinsics. Then transforms it to the camera frame using
        the tf2 api. Once transformed, it will do a pixel-by-pixel comparision
        to ground truth.

        Returns a bool indicating the the success of the computation.
        """
        rdis = np.array(self.bridge_.imgmsg_to_cv2(self.rdis_))
        uvec = np.array(self.bridge_.imgmsg_to_cv2(self.uvec_))
        cloud = np.array(self.bridge_.imgmsg_to_cv2(self.cloud_))
        conf = np.array(self.bridge_.imgmsg_to_cv2(self.conf_))

        # split out the camera-computed image planes
        x_cam = cloud[:,:,0]
        y_cam = cloud[:,:,1]
        z_cam = cloud[:,:,2]
        if ((cloud.dtype == np.float32) or
            (cloud.dtype == np.float64)):
            # convert to mm
            x_cam *= 1000.
            y_cam *= 1000.
            z_cam *= 1000.

            # cast to int16_t
            x_cam = x_cam.astype(np.int16)
            y_cam = y_cam.astype(np.int16)
            z_cam = z_cam.astype(np.int16)
        else:
            # camera data are already in mm
            pass

        # Get the unit vectors
        ex = uvec[:,:,0]
        ey = uvec[:,:,1]
        ez = uvec[:,:,2]

        # translation vector from extrinsics
        tx = self.extrinsics_.tx
        ty = self.extrinsics_.ty
        tz = self.extrinsics_.tz

        # Cast the radial distance image to float
        rdis_f = rdis.astype(np.float32)
        if (rdis.dtype == np.float32):
            # assume rdis was in meters, convert to mm
            rdis_f *= 1000.

        # Compute Cartesian
        x_ = ex * rdis_f + tx
        y_ = ey * rdis_f + ty
        z_ = ez * rdis_f + tz

        # mask out bad pixels from our computed cartesian values
        bad_mask = (np.bitwise_and(conf, 0x1) == 0x1)
        x_[bad_mask] = 0.
        y_[bad_mask] = 0.
        z_[bad_mask] = 0.

        # Transform to target frame
        #
        # NOTE: This could obviously be vectorized if we exploit our apriori
        # knowledge of the transform, but we want to test the transform we are
        # broadcasting via tf and the tf2 api for doing the transformation.
        #
        # I agree, this loop is slow and ugly :-( -- mostly b/c using the tf2
        # interface with our data structures here is kind of wonky
        #
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        n_rows = x_.shape[0]
        n_cols = x_.shape[1]
        x_f = np.zeros((n_rows, n_cols), dtype=np.float32)
        y_f = np.zeros((n_rows, n_cols), dtype=np.float32)
        z_f = np.zeros((n_rows, n_cols), dtype=np.float32)
        for i in range(n_rows):
            for j in range(n_cols):
                p = PointStamped()
                p.header = self.rdis_.header
                p.point.x = x_[i,j]
                p.point.y = y_[i,j]
                p.point.z = z_[i,j]
                pt = tf_buffer.transform(p, self.cloud_.header.frame_id,
                                         rospy.Duration(2.0))
                x_f[i,j] = pt.point.x
                y_f[i,j] = pt.point.y
                z_f[i,j] = pt.point.z

        # cast to the data type of the point cloud
        x_i = x_f.astype(np.int16)
        y_i = y_f.astype(np.int16)
        z_i = z_f.astype(np.int16)

        tol = 10 # milli-meters
        x_mask = np.fabs(x_i - x_cam) > tol
        y_mask = np.fabs(y_i - y_cam) > tol
        z_mask = np.fabs(z_i - z_cam) > tol

        # we are asserting that no pixels are out-of-tolerance
        # per the computation of the x_,y_,z_mask variables above.
        self.assertTrue(x_mask.sum() == 0)
        self.assertTrue(y_mask.sum() == 0)
        self.assertTrue(z_mask.sum() == 0)

        # if any of the above asserts fail, the test will error out,
        # so, we return True here carte blanche
        return True

def main():
    rostest.rosrun('ifm3d', 'test_camera', TestCamera, sys.argv)
    return 0

if __name__ == '__main__':
    sys.exit(main())

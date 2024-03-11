#!/usr/bin/env python3
import cv2
import sys
import time
import os
import numpy as np
import rospy
import yaml
import json
import rospkg
import pprofile
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../robot/scripts/')
from ur5_kinematics import ur5

from sensor_msgs.msg import JointState
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import ManipulatedObject
from unity_msgs.msg import HomographyMtx
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String


from __future__ import print_function

import sys
import rospy
from scene_calibration import *

def get_points(x, y):
    rospy.wait_for_service('/server_rgb_points')
    try:
        get_pts = rospy.ServiceProxy('/server_rgb_points', PointsDepthMap)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
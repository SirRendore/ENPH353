#!/usr/bin/env python

import numpy as np
import tensorflow as tf
from tensorflow import keras
import rospy

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

bridge = CvBridge()
from sensor_msgs.msg import Image
import time

rospy.init_node('plate_eader')

sub_camera_CNN = rospy.Subscriber(
    "/R1/pi_camera/image_raw", Image, plate_frame_callback)


#!/usr/bin/env python
# -*- coding: utf-8 -*-


from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Pose, TwistStamped, Vector3
from std_msgs.msg import Header
import rospy
import serial
import time
from motors import Motors

class Blimp:
  """
  Principle Class of Blimp
  """
  def __init__(self):
    self.motors = Motors()
    


#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from blimp import Blimp
from std_msgs.msg import Float64

class obstacle_check:
  def __init__(self):
    rospy.Subscriber("/ultrasonic",Float64,self.obstacle_control)
    self.blimp = Blimp()
    
  def obstacle_control(self,data):
    U=int(data.data)
    #print(U)
    if U<70:
      self.blimp.motors.forward_stop()
      self.blimp.motors.backward_stop()
      self.blimp.motors.upward_stop()    
      self.blimp.motors.downward_stop()       
    else:
       print("exit loop")
      
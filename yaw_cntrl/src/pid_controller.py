#!/usr/bin/env python
# -*- coding: utf-8 -*-



class PID_Controller:
  def __init__(self , kp,ki,kd):
      self.prev_error = 0
      self.kp = kp
      self.ki = ki
      self.kd = kd
      self.cum_error = 0
      self.i_windup = 100
  
  @staticmethod
  def range_converter(OldValue,OldMin,OldMax,NewMin,NewMax):
    OldRange = (OldMax - OldMin)
    if (OldRange == 0):
        NewValue = NewMin
    else:
        NewRange = (NewMax - NewMin)  
        NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    return NewValue
    
  def corrective_action(self,set_point,current):
    print("Setpoint is :", set_point)
    print("Current value is :", current)
    error = set_point - current

    print("Current Error: "+repr(error))
    del_error = error - self.prev_error
    self.cum_error = self.i_windup if error + self.cum_error > self.i_windup else error + self.cum_error
    print("Error: "+repr(error))
    mv = abs(self.kp * error + self.ki * self.cum_error + self.kd * del_error)
    corrected_speed = PID_Controller.range_converter(mv,0,360,0,100)
    print("Corrected Speed: "+repr(corrected_speed))
    c_180 = (current + 180) % 360
    corrected_speed = corrected_speed if set_point > current and set_point < c_180 else -corrected_speed
    print("Corrected Speed: "+repr(corrected_speed))
    return corrected_speed

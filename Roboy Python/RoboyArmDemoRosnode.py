import sys
import rospy
import time
import math

# Import Roboy Stuff
from RoboyMotorUtil import *
from RoboyShoulderMap import *

from std_msgs.msg import Float64


topic_slope = '/slope'


def receivedSlope(data):
  slope = data.data
  
  deg = math.atan(slope) / math.pi * 180.0
  print 'Slope is %f' % slope
  print 'Angle is %f' % deg
  motorPos = getMotorPositionsForAngle(deg)
  
  print 'Setting Motor Positions'
  for m in motors_shoulder:
    setMotorSetpoint(m, motorPos[m])


def startArmROS():
  init()
  print 'Initializing Motors'
  initMotor(motors_shoulder, controlMode_Force)
  initMotor(motors_elbow, controlMode_Force)
  
  print 'Setting default motor forces'
  setMotorSetpoint(motors_shoulder, 10)
  setMotorSetpoint(motors_elbow, 5)
  
  print 'Initializing Motors to Position mode'
  initMotor(motors_shoulder, controlMode_Position)
  
  rospy.Subscriber(
      topic_slope,
      Float64,
      receivedSlope)
#


if __name__ == '__main__':
  try:
    startArmROS()
    rospy.spin()
  except KeyboardInterrupt, e:
    pass
  print "exiting"
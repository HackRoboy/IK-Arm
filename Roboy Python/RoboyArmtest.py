import sys
import rospy
import time

# Import Roboy Stuff
from RoboyMotorUtil import *
from RoboyShoulderMap import *


def moveToAngle(deg):
  motorPos = getMotorPositionsForAngle(deg)
  
  print 'Initializing Motors'
  initMotor(motors_shoulder, controlMode_Position)
  
  print 'Setting Motor Positions'
  for m in motors_shoulder:
    setMotorSetpoint(m, motorPos[m])


def initArmTest():
  init()
  print 'Initializing Motors'
  initMotor(motors_shoulder, controlMode_Force)
  initMotor(motors_elbow, controlMode_Force)
  
  print 'Setting default motor forces'
  setMotorSetpoint(motors_shoulder, 10)
  setMotorSetpoint(motors_elbow, 5)
#
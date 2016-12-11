import sys
import rospy
import time
import csv

# Import Roboy Stuff
from RoboyMotorUtil import *


motorsToRecord = motors_shoulder

recordedData = []


def record(angle):
  dataset = [angle]
  for m in motorsToRecord:
    dataset.append(getMotorStatus(m)['position'])
  recordedData.append(dataset)
#


def saveRecord():
  with open('ShoulderPositionMap.csv', 'wb') as f:
    writer = csv.writer(f)
    writer.writerows(recordedData)
#


def initRecord():
  init()
  print 'Initializing Motors'
  initMotor(motors_shoulder, controlMode_Force)
  initMotor(motors_elbow, controlMode_Force)
  
  print 'Setting default motor forces'
  setMotorSetpoint(motors_shoulder, 10)
  setMotorSetpoint(motors_elbow, 5)
  
  print 'Preparing Dataset'
  dataset = ['deg']
  for m in motorsToRecord:
    dataset.append(str(m))
  recordedData.append(dataset)
#
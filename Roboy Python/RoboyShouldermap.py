import sys
import rospy
import time
import csv
from scipy.interpolate import interp1d

# Import Roboy Stuff
from RoboyMotorUtil import *


# Read measured Data from CSV

ShoulderDataPoints = []

with open('ShoulderPositionMap.csv', 'rb') as csvfile:
  reader = csv.reader(csvfile)
  for row in reader:
    ShoulderDataPoints.append(row)
#


ShoulderDataPointsDegs = []
ShoulderDataPointsMotors = {}

for d in ShoulderDataPoints:
  if d[0] == 'deg':
    for i in range(1, len(d)):
      ShoulderDataPointsMotors[d[i]] = []
    continue
  ShoulderDataPointsDegs.append(float(d[0])) # Append degree value
  for i in range(1, len(d)):
    ShoulderDataPointsMotors[ShoulderDataPoints[0][i]].append(float(d[i])) # Associative Array Motornum => Array of Datapoints

ShoulderInterpolators = {}
for m in motors_shoulder:
  ShoulderInterpolators[m] = interp1d(ShoulderDataPointsDegs, ShoulderDataPointsMotors[str(m)], kind='linear', fill_value='extrapolate')

def getMotorPositionsForAngle(deg):
  motorPos = {}
  for m in motors_shoulder:
    motorPos[m] = ShoulderInterpolators[m](deg)
  return motorPos
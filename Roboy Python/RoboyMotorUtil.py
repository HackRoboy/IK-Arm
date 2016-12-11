import sys
import rospy
import time

# Import Roboy messages
from common_utilities.msg import *
from common_utilities.srv import *

from sensor_msgs.msg import JointState


# CONSTANTS

NodeName = 'RoboyPyClient'

service_roboy_init = '/roboy/initialize'
topic_roboy_motor_status = '/roboy/status_motor%i'
topic_roboy_motor_setpoint = '/roboy/control_me/setpoint'
topic_joint_states = '/joint_states'
motor_name_format = "motor%i"

controlMode_Position = 1
controlMode_Velocity = 2
controlMode_Force = 3

motors_shoulder = [
  1,  # G0 M1; Front low 1
  2,  # G0 M2; Front up  2 (Arm)
  4,  # G1 M1; Top
  6,  # G1 M2; Back  upper
  11, # G1 M3; Back  middle
  5,  # G0 M1; Back  lower
]

motors_elbow = [
  10,  # G2 M2; Biceps
  8,  # G2 M0; Triceps
]


# VARIABLES

motorSetpointPublisher = rospy.Publisher(topic_roboy_motor_setpoint, MotorCommand, queue_size=10)

jointState_names = []
jointState_position = []
jointState_velocity = []
jointState_effort = []


def receivedMotorStatus(data, mNum):
  #print( 'Received Status info on Motor %i' % mNum )
  pass
#


def receivedJointState(data):
  global jointState_names, jointState_position, jointState_velocity, jointState_effort
  try:
    jointState_names = data.name
    jointState_position = data.position
    jointState_velocity = data.velocity
    jointState_effort = data.effort
  except Exception as e:
    print 'Error:'
    print e
    pass
#


def getMotorStatus(num):
  try:
    i = jointState_names.index(motor_name_format % num)
    return {
      'name': jointState_names[i],
      'position': jointState_position[i],
      'velocity': jointState_velocity[i],
      'effort': jointState_effort[i],
    }
  except Exception as e:
    print e
    print jointState_names
    return False



def initMotor(nums, modes):
  
  if(type(nums) is not list):
    nums = [nums]
  
  if(type(modes) is not list):
    modes = [modes] * len(nums)
  
  rospy.wait_for_service(service_roboy_init)
  
  # Listen to all Motor states
  for n in nums:
    rospy.Subscriber(
      topic_roboy_motor_status % n,
      ControllerState,
      receivedMotorStatus,
      n)
  
  
  try:
    InitService = rospy.ServiceProxy(service_roboy_init, Initialize)
    print "Initializing Motors %s as %s" % (str(nums), str(modes))
    result = InitService(nums, modes)
    print "Result was: %s" % str(result)
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e
#


def setMotorSetpoint(num, value):
  # Automatically call self recursively if list
  if(type(num) is list):
    if(type(value) is not list):
      value = [value] * len(num)
    for n, v in zip(num, value):
      setMotorSetpoint(n, v)
    return
  
  print "Set Motor %i Setpoint to %f" % (num, value)
  mc = MotorCommand(num, value)
  motorSetpointPublisher.publish(mc)
#



def init():
  rospy.init_node(NodeName, anonymous=True)
  rospy.Subscriber(
      topic_joint_states,
      JointState,
      receivedJointState)
  time.sleep(1)
#


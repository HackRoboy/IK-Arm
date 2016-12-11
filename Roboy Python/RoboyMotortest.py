import sys
import rospy
import time

# Import Roboy messages
from common_utilities.msg import *
from common_utilities.srv import *

NodeName = 'RoboyPyTester'

service_roboy_init = '/roboy/initialize'
topic_roboy_motor_status = '/roboy/status_motor%i'
topic_roboy_motor_setpoint = '/roboy/control_me/setpoint'

controlMode_Position = 1
controlMode_Velocity = 2
controlMode_Force = 3

motors_shoulder = [
  1,  # G0 M1; Front low 1
  2,  # G0 M2; Front up  2
  4,  # G1 M1; Top
  6,  # G1 M2; Back  upper
  11, # G1 M3; Back  middle
  5,  # G0 M1; Back  lower
]

motors_elbow = [
  10,  # G2 M2; Biceps
  8,  # G2 M0; Triceps
]


motorSetpointPublisher = rospy.Publisher(topic_roboy_motor_setpoint, MotorCommand, queue_size=10)




def receivedMotorStatus(data, mNum):
  print( 'Received Status info on Motor %i' % mNum )


def initMotors(nums, modes):
  
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




def motorTest():
  
  print 'Initializing Motor'
  initMotors(10, controlMode_Force)
  print 'Setting motor force value'
  
  setMotorSetpoint(motors_shoulder, 10)
  setMotorSetpoint(motors_elbow, 5)
  
  rospy.spin()
#



def init():
  rospy.init_node(NodeName, anonymous=True)
  time.sleep(1)



if __name__ == '__main__':
  init()
  try:
    motorTest()
  except KeyboardInterrupt, e:
    pass
  print "exiting"
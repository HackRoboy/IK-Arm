#include "common_utilities/Initialize.h"
#include "common_utilities/MotorCommand.h"
#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <flexrayusbinterface/CommunicationData.h>

class MyoMotor {
public:
  MyoMotor() {
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "control_me", ros::init_options::NoSigintHandler);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    initialize_srv = nh->advertiseService(
        "/roboy/initialize", &MyoMotor::initializeControllers, this);
    control_me_sub = nh->subscribe("/roboy/control_me/setpoint", 100,
                                   &MyoMotor::controlMe, this);
    non_realtime_loop =
        nh->createTimer(ros::Duration(0.02), &MyoMotor::updateData, this);
    pubber = nh->advertise<sensor_msgs::JointState>("joint_states", 10);
  }

  void updateData(const ros::TimerEvent &e) {
    sensor_msgs::JointState msg;
    flexray.exchangeData();
    msg.header.stamp = ros::Time::now();
    for (auto const &motorInst : active_motors_) {
      uint ganglion = motorInst.first / 4;
      uint motor = motorInst.first % 4;
      std::stringstream motorname;
      motorname << "motor" << motorInst.first;
      msg.name.push_back(motorname.str());
      msg.position.push_back(
          flexray.GanglionData[ganglion].muscleState[motor].actuatorPos *
          flexray.controlparams.radPerEncoderCount);
      msg.velocity.push_back(
          flexray.GanglionData[ganglion].muscleState[motor].actuatorVel *
          flexray.controlparams.radPerEncoderCount);
      msg.effort.push_back(
          flexray.GanglionData[ganglion].muscleState[motor].tendonDisplacement /
          32768.0f);
	pubber.publish(msg);
    }
  }

  bool initializeControllers(common_utilities::Initialize::Request &req,
                             common_utilities::Initialize::Response &res) {
    uint i = 0;
    for (auto id : req.idList) {
      uint ganglion = id / 4;
      uint motor = id % 4;
      switch ((uint)req.controlmode[i]) {
      case 1: {
        ROS_INFO("position controller ganglion %d motor %d", ganglion, motor);
        flexray.initPositionControl(ganglion, motor);

        break;
      }
      case 2: {
        ROS_INFO("velocity controller ganglion %d motor %d", ganglion, motor);
        flexray.initVelocityControl(ganglion, motor);
        break;
      }
      case 3: {
        ROS_INFO("force controller ganglion %d motor %d", ganglion, motor);
        flexray.initForceControl(ganglion, motor);
        break;
      }
      default:
        ROS_WARN("The requested controlMode is not available, "
                 "choose [1]PositionController [2]VelocityController "
                 "[3]ForceController");
        break;
      }
      active_motors_[id] = req.controlmode[i];
      i++;
    }
    return true;
  }

  void controlMe(const common_utilities::MotorCommand::ConstPtr &msg) {
    uint ganglion = msg->id / 4;
    uint motor = msg->id % 4;
    ROS_INFO("Setpoint of ganglion %d motor %d to %f", ganglion, motor,
             msg->setpoint);
    if (ganglion < 3)
      flexray.commandframe0[ganglion].sp[motor] = msg->setpoint;
    else
      flexray.commandframe1[ganglion - 3].sp[motor] = msg->setpoint;
    flexray.updateCommandFrame();
    flexray.exchangeData();
  }

  /*
  * Setup motor in position, velocity or force control mode.
  * position control mode: 0
  * velocity control mode: 1
  * effort / force control mode: 2
  */
  bool setupMotor(unsigned int controlMode) {
    enum controllerOptions { position, velocity, effort = 2, force = 2 };
    switch (controlMode) {
    case position:
      flexray.initPositionControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in position control mode.");
      break;
    case velocity:
      flexray.initVelocityControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in velocity control mode.");
      break;
    case effort:
      flexray.initForceControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in force control mode.");
      break;
    default:
      ROS_ERROR("control_me: Received an unknown control mode. Check the enum: "
                "position control mode: 0, velocity control mode: 1, effort / "
                "force control mode: 2 ");
      return false;
    }
    flexray.exchangeData();
    return true;
  }

  /*
  * Implements the service to move the motors.
  */
  bool moveMotor(double setpoint) {
    if (flexray.commandframe0[0].sp[0] = setpoint) {
      flexray.updateCommandFrame();
      flexray.exchangeData();
      return true;
    }
    return false;
  }

  double readDisplacementSensor(void) {
    flexray.exchangeData();
    return flexray.GanglionData[0].muscleState[0].tendonDisplacement / 32768.0f;
  }

private:
  FlexRayHardwareInterface flexray;
  ros::NodeHandlePtr nh;
  ros::ServiceServer initialize_srv;
  ros::Subscriber control_me_sub;
  ros::Timer non_realtime_loop;
  std::map<unsigned int, unsigned int> active_motors_;
  ros::Publisher pubber;
};

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line.
   * For programmatic remappings you can use a different version of init() which
   * takes
   * remappings directly, but for most command-line programs, passing argc and
   * argv is
   * the easiest way to do it.  The third argument to init() is the name of the
   * node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "Roboy_Control_Me");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  MyoMotor motor;
  ros::spin();
}

#include <torso_interface/torso_interface.h>

TorsoInterface::TorsoInterface(ros::NodeHandle h) : rcomponent::RComponent(h), nh_(h), pnh_("~")
{
  component_name.assign(pnh_.getNamespace());
  rosReadParams();

  sComponentName_ = "TorsoInterface";
}

TorsoInterface::~TorsoInterface()
{
}

int TorsoInterface::rosSetup()
{
  RComponent::rosSetup();

  // PUBLISHERS

  // Publisher of the desired positions to the torso controller
  torso_controller_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("torso_controllers/joint_trajectory", 1);

  // SERVICES

  // Service to set elevator raised
  set_elevator_raised_ = pnh_.advertiseService("set_elevator_raised", &TorsoInterface::setElevatorRaisedCb, this);

  //! Service to set elevator position
  set_elevator_ = pnh_.advertiseService("set_elevator", &TorsoInterface::setElevatorCb, this);

  //! Service to set head pan position
  set_pan_ = pnh_.advertiseService("set_head_pan", &TorsoInterface::setPanCb, this);

  //! Service to set head tilt position
  set_tilt_ = pnh_.advertiseService("set_head_tilt", &TorsoInterface::setTiltCb, this);
}

int TorsoInterface::rosShutdown()
{
  RComponent::rosShutdown();
}

void TorsoInterface::rosReadParams()
{
  RComponent::rosReadParams();

  bool required = true;
  bool not_required = false;

  readParam(pnh_, "desired_freq", desired_freq_, DEFAULT_FREQ, not_required);
}

void TorsoInterface::rosPublish()
{
  RComponent::rosPublish();

  if (state == robotnik_msgs::State::READY_STATE)
  {
  }
}

void TorsoInterface::initState()
{
  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void TorsoInterface::standbyState()
{
  /*
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }
  else
  {
    switchToState(robotnik_msgs::State::READY_STATE);
  }
  */
}

void TorsoInterface::readyState()
{
  /*
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }
  */
}

void TorsoInterface::emergencyState()
{
  if (checkTopicsHealth() == true)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

void TorsoInterface::failureState()
{
}

/*!	\fn bool
 * TorsoInterface::setElevatorRaised(std_srvs::SetBool::Request& request,
 *                                             std_srvs::SetBool::Response& response)
 * 	\brief ROS service callback to set the elevator raised
 */
bool TorsoInterface::setElevatorRaisedCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_slider_joint");

  trajectory_msgs::JointTrajectoryPoint position;
  if (request.data == true)
  {
    position.positions.push_back(MAX);
  }
  else
  {
    position.positions.push_back(MIN);
  }

  joint_trajectory.points.push_back(position);

  torso_controller_pub_.publish(joint_trajectory);

  response.success = true;
  response.message = "Message sent to controller";

  return true;
}

//! Set Elevator Service Cb
bool TorsoInterface::setElevatorCb(robotnik_msgs::set_float_value::Request& request,
                                   robotnik_msgs::set_float_value::Response& response)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_slider_joint");

  trajectory_msgs::JointTrajectoryPoint position;

  position.positions.push_back(request.value);

  joint_trajectory.points.push_back(position);

  torso_controller_pub_.publish(joint_trajectory);

  response.ret = true;
  std_msgs::String msg;
  msg.data = "Message sent to controller";
  response.errorMessage = msg;

  return true;
}

//! Set Pan Service Cb
bool TorsoInterface::setPanCb(robotnik_msgs::set_float_value::Request& request,
                              robotnik_msgs::set_float_value::Response& response)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_pan_joint");

  trajectory_msgs::JointTrajectoryPoint position;

  position.positions.push_back(request.value);

  joint_trajectory.points.push_back(position);

  torso_controller_pub_.publish(joint_trajectory);

  response.ret = true;
  std_msgs::String msg;
  msg.data = "Message sent to controller";
  response.errorMessage = msg;

  return true;
}

//! Set Tilt Service Cb
bool TorsoInterface::setTiltCb(robotnik_msgs::set_float_value::Request& request,
                               robotnik_msgs::set_float_value::Response& response)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_tilt_joint");

  trajectory_msgs::JointTrajectoryPoint position;

  position.positions.push_back(request.value);

  joint_trajectory.points.push_back(position);

  torso_controller_pub_.publish(joint_trajectory);

  response.ret = true;
  std_msgs::String msg;
  msg.data = "Message sent to controller";
  response.errorMessage = msg;

  return true;
}
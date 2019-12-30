/*! \class RComponent
 *  \file RComponent.h
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2014
 *  \brief Class to define a standard and shared structure (attributes & methods) for all the components
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _TORSO_INTERFACE_
#define _TORSO_INTERFACE_

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>

#include <rcomponent/rcomponent.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <robotnik_msgs/set_float_value.h>

//! Size of string for logging
#define DEFAULT_FREQ 5.0
#define MAX std::numeric_limits<double>::max()
#define MIN std::numeric_limits<double>::min()

using namespace std;

//! Class Rcomponent
class TorsoInterface : public rcomponent::RComponent
{
public:
  TorsoInterface(ros::NodeHandle h);
  virtual ~TorsoInterface();

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  //! Actions performed on init state
  virtual void initState();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();

  /* RComponent stuff !*/

  //! Set Elevator Raised Service Cb
  bool setElevatorRaisedCb(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  //! Set Elevator Service Cb
  bool setElevatorCb(robotnik_msgs::set_float_value::Request& request,
                     robotnik_msgs::set_float_value::Response& response);
  //! Set Pan Service Cb
  bool setPanCb(robotnik_msgs::set_float_value::Request& request, robotnik_msgs::set_float_value::Response& response);
  //! Set Tilt Service Cb
  bool setTiltCb(robotnik_msgs::set_float_value::Request& request, robotnik_msgs::set_float_value::Response& response);

public:
protected:
  /* ROS stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  // Publishers

  //! To publish the desired positions to the torso controller
  ros::Publisher torso_controller_pub_;

  // Subscribers

  // Services

  //! Service to raise elevator
  ros::ServiceServer set_elevator_raised_;
  //! Service to set elevator position
  ros::ServiceServer set_elevator_;
  //! Service to set head pan position
  ros::ServiceServer set_pan_;
  //! Service to set head tilt position
  ros::ServiceServer set_tilt_;

  /* ROS stuff !*/

  //!	Store the component name
  std::string sComponentName_;

};  // namespace _TORSO_INTERFACE_

#endif

/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/testhehe/qnode.hpp"
#include "ui_main_window.h"
#include <iostream>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace testhehe {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, Ui::MainWindowDesign* pui ) :
	init_argc(argc),
  init_argv(argv),
  ui(pui)
{
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"testhehe");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  speed_publisher   = n.advertise<geometry_msgs::Twist>("/robot/speed",1000);
  odm_reset_publisher = n.advertise<geometry_msgs::Pose2D>("/robot/odm_reset",1000);
  control_publisher = n.advertise<std_msgs::Bool>("/robot/controlSignal",1000);
  odmSub = n.subscribe("/robot_odm",10,&QNode::odmCallback, this);
  imuSub = n.subscribe("/imu_angle",10,&QNode::imuCallback, this);
  speed_flag = 0;
  controlSignal.data = false;
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"testhehe");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  speed_publisher   = n.advertise<geometry_msgs::Twist>("/robot/speed",1000);
  odm_reset_publisher = n.advertise<geometry_msgs::Pose2D>("/robot/odm_reset",1000);
  control_publisher = n.advertise<std_msgs::Bool>("/robot/controlSignal",1000);
  odmSub = n.subscribe("/robot_odm",10,&QNode::odmCallback, this);
  speed_flag = 0;
  controlSignal.data = false;
	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
	int count = 0;
  std_msgs::Bool flag_msg;
  flag_msg.data = true;
	while ( ros::ok() ) {
    geometry_msgs::Twist speed_msg;
    geometry_msgs::Pose2D resetPose;
    switch (speed_flag) {
    case 0:
      speed_msg.angular.z =0;
      speed_msg.linear.x = 0;
      speed_publisher.publish(speed_msg);
      break;
    case 1:
      speed_msg.angular.z =0;
      speed_msg.linear.x = 0.2;
      speed_publisher.publish(speed_msg);
      break;
    case 2:
      speed_msg.angular.z =0;
      speed_msg.linear.x = -0.2;
      speed_publisher.publish(speed_msg);
      break;
    case 3:
      speed_msg.angular.z =0.3;
      speed_msg.linear.x = 0;
      speed_publisher.publish(speed_msg);
      break;
    case 4:
      speed_msg.angular.z =-0.3;
      speed_msg.linear.x = 0;
      speed_publisher.publish(speed_msg);
      break;
    case 5:   // Set speeds by other programs who send message to the speed_publisher
      if( controlSignal.data == false )
          controlSignal.data = true;
      else
          controlSignal.data = false;
      control_publisher.publish(controlSignal);
      speed_flag = 7;
      break;
    case 6:
      resetPose.x = resetOdmXloc;
      resetPose.y = resetOdmYLoc;
      resetPose.theta = resetOdmTheta;
      odm_reset_publisher.publish(resetPose);
      speed_flag = 7;
      speed_publisher.publish(speed_msg);
      break;
    default:
      break;
    }
    //log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace testhehe

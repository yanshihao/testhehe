/**
 * @file /include/testhehe/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef testhehe_QNODE_HPP_
#define testhehe_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include "ui_main_window.h"
#include <iostream>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace testhehe {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
  QNode(int argc, char** argv , Ui::MainWindowDesign* ui);
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };
   /*********************
   ** Logging
   **********************/
  enum SpeedFlag {
           STOP,
           UP,
           DOWN,
           LEFT,
           RIGHT,
           CONTROL,
           RESET
  };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  void setSpeedFlag(SpeedFlag f1) {speed_flag = f1; }
  void setResetOdm(double x, double y, double theta)
  {
       resetOdmXloc = x;
       resetOdmYLoc = y;
       resetOdmTheta = theta;
  }
  void odmCallback( const geometry_msgs::Pose2DPtr robotPose )
  {
      Q_EMIT getOdmInformation(robotPose->x, robotPose->y, robotPose->theta);
  }

  // use signal and slot
  void imuCallback( const geometry_msgs::Vector3ConstPtr imuPose )
  {
      Q_EMIT getImuInformation(imuPose->x, imuPose->y, imuPose->z);
  }
Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void getImuInformation(double x, double y, double z);
  void getOdmInformation(double x, double y, double theta);

private:
	int init_argc;
	char** init_argv;
  int speed_flag;    // SpeedFlag
  ros::Publisher odm_reset_publisher;
  ros::Publisher speed_publisher;
  ros::Publisher control_publisher;
  ros::Subscriber odmSub;
  ros::Subscriber imuSub;
  QStringListModel logging_model;
  double resetOdmXloc;
  double resetOdmYLoc;
  double resetOdmTheta;
  std_msgs::Bool controlSignal;
  Ui::MainWindowDesign* ui;
};

}  // namespace testhehe

#endif /* testhehe_QNODE_HPP_ */

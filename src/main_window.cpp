/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <vector>
#include "../include/testhehe/main_window.hpp"

using namespace std;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace testhehe {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv,&ui)
  , m_nTimerID(0)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(&qnode, SIGNAL(getImuInformation(double,double,double)), this, SLOT(on_get_imu_information(double,double,double)));
    QObject::connect(&qnode, SIGNAL(getOdmInformation(double,double,double)), this, SLOT(on_get_odm_information(double,double,double)));
    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "testhehe");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "testhehe");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}


}  // namespace testhehe

void testhehe::MainWindow::on_stop_button_clicked()
{
    qnode.setSpeedFlag( QNode::STOP);
}

void testhehe::MainWindow::on_upButton_clicked()
{
    qnode.setSpeedFlag(QNode::UP);
}

void testhehe::MainWindow::on_down_Button_clicked()
{
     qnode.setSpeedFlag(QNode::DOWN);
}

void testhehe::MainWindow::on_left_Button_clicked()
{
     qnode.setSpeedFlag(QNode::LEFT);
}

void testhehe::MainWindow::on_right_Button_clicked()
{
     qnode.setSpeedFlag(QNode::RIGHT);
}

void testhehe::MainWindow::on_contrl_Button_clicked()
{
    if(ui.button_connect->isEnabled() == true)
        return;
    if( ui.contrl_Button->text() == "Start Control" )
    {
        ui.contrl_Button->setText("Stop Control");
        ui.timeLeftEdit->setText(ui.timeSetEdit->text());
        m_nTimerID = startTimer(1000);
    }
    else
    {
        ui.contrl_Button->setText("Start Control");
        ui.timeLeftEdit->setText(QString::number(0));
        if( m_nTimerID != 0 )
            killTimer((m_nTimerID));
    }
    qnode.setSpeedFlag(QNode::CONTROL);
}

void testhehe::MainWindow::on_reset_odm_clicked()
{
    getResetOdm();
    qnode.setSpeedFlag(QNode::RESET);
}

void testhehe::MainWindow::getResetOdm()
{
    double x = ui.resetOdmX->text().toDouble();
    double y = ui.resetOdmY->text().toDouble();
    double theta = ui.resetOdmTheta->text().toDouble();
    qnode.setResetOdm(x, y, theta);
}


void testhehe::MainWindow::on_get_imu_information(double x, double y, double z)
{
    ui.imuXEdit->setText(QString::number(x, 10, 4));
    ui.imuYEdit->setText(QString::number(y, 10, 4));
    ui.imuZEdit->setText(QString::number(z, 10, 4));
}

void testhehe::MainWindow::on_get_odm_information(double x, double y, double theta)
{
    ui.odmXEdit->setText(QString::number(x, 10, 4));
    ui.odmYEdit->setText(QString::number(y, 10, 4));
    ui.odmThetaEdit->setText(QString::number(theta, 10, 4));
}

void testhehe::MainWindow::timerEvent(QTimerEvent *event)
 {
     if(event->timerId() == m_nTimerID){
         int nsec = ui.timeLeftEdit->text().toInt() - 1;
         if( nsec <= 0)
         {
             ui.timeLeftEdit->setText(QString::number(0));
             ui.contrl_Button->setText("Start Control");
             killTimer((m_nTimerID));
             m_nTimerID = 0;
             qnode.setSpeedFlag(QNode::CONTROL);
         }
         else
             ui.timeLeftEdit->setText(QString::number(nsec));
     }
 }

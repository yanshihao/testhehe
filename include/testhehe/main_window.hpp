/**
 * @file /include/testhehe/main_window.hpp
 *
 * @brief Qt based gui for testhehe.
 *
 * @date November 2010
 **/
#ifndef testhehe_MAIN_WINDOW_H
#define testhehe_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
namespace testhehe {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
  virtual void timerEvent( QTimerEvent *event);
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
  void on_checkbox_use_environment_stateChanged(int state);
  void on_get_odm_information(double x, double y, double theta);
  void on_get_imu_information(double x, double y, double z);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private Q_SLOTS:
    void on_stop_button_clicked();

    void on_upButton_clicked();

    void on_down_Button_clicked();

    void on_left_Button_clicked();

    void on_right_Button_clicked();

    void on_contrl_Button_clicked();

    void on_reset_odm_clicked();

    void getResetOdm();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  int m_nTimerID;
};

}  // namespace testhehe

#endif // testhehe_MAIN_WINDOW_H

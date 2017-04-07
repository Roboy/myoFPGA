/**
 * @file /include/interface/main_window.hpp
 *
 * @brief Qt based gui for interface.
 *
 * @date November 2010
 **/
#ifndef interface_MAIN_WINDOW_H
#define interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "roboy_managing_node/myoMaster.hpp"
#define RUN_IN_THREAD
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace interface {

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
	void showNoMasterMessage();
    MyoMaster *myoMaster;
private:
    void MotorStatus(const communication::MotorStatus::ConstPtr &msg);
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    void updateSetPoints(int percent);
	void updateControllerParams();

    /******************************************
    ** Manual connections
    *******************************************/
    void plotData();
Q_SIGNALS:
    void newData();
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig;
    ros::Subscriber motorStatus;
    QVector<double> time;
    map<int,QVector<double>> motorData[14];
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H

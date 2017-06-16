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
#include <QFileSystemModel>
#include "ui_main_window.h"
#include "roboy_managing_node/myoMaster.hpp"
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/JointCommand.h>
#include <roboy_communication_middleware/ArucoPose.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <fstream>
#include <thread>
#include <sensor_msgs/Image.h>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>

#define RUN_IN_THREAD
#define NUMBER_OF_FPGAS 5
#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_JOINT_SENSORS 4

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

enum PLOTDATA{ MOTOR, JOINT};

using namespace std;
using namespace Eigen;

namespace interface {

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
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    void MotorRecordPack(const roboy_communication_middleware::MotorRecord::ConstPtr &msg);
    void DisplayImage(const sensor_msgs::ImageConstPtr &msg);
    void JointCommand(const roboy_communication_middleware::JointCommand::ConstPtr& msg);
    void ArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr& msg);
public Q_SLOTS:
	void on_actionAbout_triggered();
    void updateSetPointsMotorControl(int percent);
    void updateSetPointsMotorControlAll(int percent);
    void updateSetPointsJointControl(int percent);
    void updateSetPointsJointControlAll(int percent);
	void updateControllerParams();
    void movementPathChanged();
    void recordMovement();
    void plotData(int id);
    bool playMovement();
    void stopMovement();
    void rewindMovement();
    void pauseMovement();
    void loopMovement();
    void stopButtonClicked();
    void danceBitch();
Q_SIGNALS:
    void newData(int id);
private:
	Ui::MainWindowDesign ui;
    ros::NodeHandlePtr nh;
    ros::Publisher motorConfig, motorRecordConfig, motorTrajectory, motorTrajectoryControl, hipCenter_pub;
    ros::Subscriber motorStatus, motorRecord, jointStatus, jointCommand, realsense, arucoPose;
    QVector<double> time;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
	QVector<double> jointData[NUMBER_OF_FPGAS][NUMBER_OF_JOINT_SENSORS][4];
	bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA], jointControl = false, motorControl = false, dance = false;
    long int counter = 0;
    int samples_per_plot = 300;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
	QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
							   Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    QFileSystemModel *model;
    int numberOfRecordsToWaitFor = 0;
    map<int, vector<int32_t>[NUMBER_OF_MOTORS_PER_FPGA]> records;
    QImage imdisplay;
    vector<float> angle, jointAngleOffset, setPointAngle;
    map<int, Vector3f> arucoMarkerPosition;
};

}  // namespace interface

#endif // interface_MAIN_WINDOW_H

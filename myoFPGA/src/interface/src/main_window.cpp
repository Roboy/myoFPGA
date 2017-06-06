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
#include "../include/interface/interface/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace interface {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

bool shouldUpdateSyncCalibrationJoints = false;
bool ignoreJointCommands = false;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(this, SIGNAL(newData(int)), this, SLOT(plotData(int)));

    QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor6, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor7, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor8, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor9, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor10, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor11, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor12, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.motor13, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControl(int)));
    QObject::connect(ui.allMotors, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsMotorControlAll(int)));
    QObject::connect(ui.joint0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
    QObject::connect(ui.joint1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
    QObject::connect(ui.joint2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
    QObject::connect(ui.joint3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl(int)));
    QObject::connect(ui.cali_joint0, SIGNAL(valueChanged(int)), this, SLOT(updateCalibrationSetPointsJointControl(int)));
    QObject::connect(ui.cali_joint1, SIGNAL(valueChanged(int)), this, SLOT(updateCalibrationSetPointsJointControl(int)));
    QObject::connect(ui.cali_joint2, SIGNAL(valueChanged(int)), this, SLOT(updateCalibrationSetPointsJointControl(int)));
    QObject::connect(ui.cali_joint3, SIGNAL(valueChanged(int)), this, SLOT(updateCalibrationSetPointsJointControl(int)));
    QObject::connect(ui.allJoints, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControlAll(int)));
    QObject::connect(ui.updateController, SIGNAL(clicked()), this, SLOT(updateControllerParams()));
    QObject::connect(ui.record, SIGNAL(clicked()), this, SLOT(recordMovement()));
    QObject::connect(ui.play, SIGNAL(clicked()), this, SLOT(playMovement()));
    QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stopMovement()));
    QObject::connect(ui.rewind, SIGNAL(clicked()), this, SLOT(rewindMovement()));
    QObject::connect(ui.pause, SIGNAL(clicked()), this, SLOT(pauseMovement()));
    QObject::connect(ui.loop, SIGNAL(clicked()), this, SLOT(loopMovement()));
    QObject::connect(ui.stop_button_motorControl, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    QObject::connect(ui.stop_button_jointControl, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    QObject::connect(ui.cali_submit_jointControl, SIGNAL(clicked()), this, SLOT(submitCalibrationJoints()));
    QObject::connect(ui.wait_button_joint_control, SIGNAL(clicked()), this, SLOT(submitWaitJoints()));
    QObject::connect(ui.run_button_joint_control, SIGNAL(clicked()), this, SLOT(submitRunJoints()));
    ui.stop_button_motorControl->setStyleSheet("background-color: red");
    ui.wait_button_joint_control->setStyleSheet("background-color: red");
    ui.run_button_joint_control->setStyleSheet("background-color: green");
    ui.stop_button_jointControl->setStyleSheet("background-color: red");
    ui.cali_submit_jointControl->setStyleSheet("background-color: green");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "interface",
                  ros::init_options::NoSigintHandler |
                          ros::init_options::AnonymousName|
                          ros::init_options::NoRosout);
    }
    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &MainWindow::MotorStatus, this);
    jointStatus = nh->subscribe("/roboy/middleware/JointStatus", 1, &MainWindow::JointStatus, this);
    motorConfig = nh->advertise<roboy_communication_middleware::MotorConfig>("/roboy/middleware/MotorConfig", 1);
    motorRecordConfig = nh->advertise<roboy_communication_middleware::MotorRecordConfig>("/roboy/middleware/MotorRecordConfig", 1);
    motorRecord = nh->subscribe("/roboy/middleware/MotorRecord", 100, &MainWindow::MotorRecordPack, this);
    motorTrajectory = nh->advertise<roboy_communication_middleware::MotorRecord>("/roboy/middleware/MotorTrajectory", 1);
    motorTrajectoryControl = nh->advertise<roboy_communication_middleware::MotorTrajectoryControl>("/roboy/middleware/MotorTrajectoryControl", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.current_plot->addGraph();
        ui.current_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
    }
    ui.position_plot->xAxis->setLabel("x");
    ui.position_plot->yAxis->setLabel("ticks");
    ui.position_plot->replot();

    ui.velocity_plot->xAxis->setLabel("x");
    ui.velocity_plot->yAxis->setLabel("ticks/s");
    ui.velocity_plot->replot();

    ui.displacement_plot->xAxis->setLabel("x");
    ui.displacement_plot->yAxis->setLabel("ticks");
    ui.displacement_plot->replot();

    ui.current_plot->xAxis->setLabel("x");
    ui.current_plot->yAxis->setLabel("mA");
    ui.current_plot->replot();

    ui.relAngle0_plot->addGraph();
    ui.relAngle0_plot->graph(0)->setPen(QPen(color_pallette[0]));
    ui.relAngle0_plot->xAxis->setLabel("x");
    ui.relAngle0_plot->yAxis->setLabel("degrees");
    ui.relAngle0_plot->yAxis->setRange(0,360);

    ui.relAngle1_plot->addGraph();
    ui.relAngle1_plot->graph(0)->setPen(QPen(color_pallette[1]));
    ui.relAngle1_plot->xAxis->setLabel("x");
    ui.relAngle1_plot->yAxis->setLabel("degrees");
    ui.relAngle1_plot->yAxis->setRange(0,360);

    ui.relAngle2_plot->addGraph();
    ui.relAngle2_plot->graph(0)->setPen(QPen(color_pallette[2]));
    ui.relAngle2_plot->xAxis->setLabel("x");
    ui.relAngle2_plot->yAxis->setLabel("degrees");
    ui.relAngle2_plot->yAxis->setRange(0,360);

    ui.relAngle3_plot->addGraph();
    ui.relAngle3_plot->graph(0)->setPen(QPen(color_pallette[3]));
    ui.relAngle3_plot->xAxis->setLabel("x");
    ui.relAngle3_plot->yAxis->setLabel("degrees");
    ui.relAngle3_plot->yAxis->setRange(0,360);

    ui.relAngle0_plot->replot();
    ui.relAngle1_plot->replot();
    ui.relAngle2_plot->replot();
    ui.relAngle3_plot->replot();

    updateControllerParams();

    model = new QFileSystemModel;
    movementPathChanged();

    shouldUpdateSyncCalibrationJoints = false;
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

void MainWindow::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg){
    ROS_INFO_THROTTLE(5, "receiving motor status");
    time.push_back(counter++);
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        motorData[msg->id][motor][0].push_back(msg->position[motor]);
        motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
        motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
        motorData[msg->id][motor][3].push_back(msg->pwmRef[motor]);
        if (motorData[msg->id][motor][0].size() > samples_per_plot) {
            motorData[msg->id][motor][0].pop_front();
            motorData[msg->id][motor][1].pop_front();
            motorData[msg->id][motor][2].pop_front();
            motorData[msg->id][motor][3].pop_front();
        }
        if(msg->current[motor]!=0) {
            motorConnected[msg->id][motor] = true;
        }else{
            motorConnected[msg->id][motor] = false;
            switch(motor) {
                case 0:   ui.setPoint_motor0->setText("dead"); break;
                case 1:   ui.setPoint_motor1->setText("dead"); break;
                case 2:   ui.setPoint_motor2->setText("dead"); break;
                case 3:   ui.setPoint_motor3->setText("dead"); break;
                case 4:   ui.setPoint_motor4->setText("dead"); break;
                case 5:   ui.setPoint_motor5->setText("dead"); break;
                case 6:   ui.setPoint_motor6->setText("dead"); break;
                case 7:   ui.setPoint_motor7->setText("dead"); break;
                case 8:   ui.setPoint_motor8->setText("dead"); break;
                case 9:   ui.setPoint_motor9->setText("dead"); break;
                case 10:   ui.setPoint_motor10->setText("dead"); break;
                case 11:   ui.setPoint_motor11->setText("dead"); break;
                case 12:   ui.setPoint_motor12->setText("dead"); break;
                case 13:   ui.setPoint_motor13->setText("dead"); break;
            }
        }
    }
    if(time.size()>samples_per_plot)
        time.pop_front();
    static int counter = 0;
    if((counter++)%3==0)
        Q_EMIT newData(MOTOR);
}

void MainWindow::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg){
    if (ignoreJointCommands) return;

    ROS_INFO_THROTTLE(5, "receiving joint status");
    float jointAngles[NUMBER_OF_JOINT_SENSORS];
    for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {
        jointAngles[joint] = msg->relAngles[joint] / 4096.0 * 360.0;
        jointData[msg->id][joint][1].push_back(jointAngles[joint]);
        if (jointData[msg->id][joint][1].size() > samples_per_plot) {
            jointData[msg->id][joint][1].pop_front();
        }
    }

    if(jointControl){
        ROS_INFO_THROTTLE(5,"joint control active");
        std::lock_guard<std::mutex> lock(myoMaster->mux);
        float error[NUMBER_OF_JOINT_SENSORS];
        float integral[NUMBER_OF_JOINT_SENSORS];
        float integral_max = 360;
        QSlider * joins[2][4] = {
                {
                        ui.joint0,
                        ui.joint1,
                        ui.joint2,
                        ui.joint3
                },
                {
                        ui.cali_joint0,
                        ui.cali_joint1,
                        ui.cali_joint2,
                        ui.cali_joint3
                }
        };
        QLineEdit * kds[2] = {
                ui.Kd_jointControl,
                ui.cali_Kd_jointControl
        };
        QLineEdit * kps[2] = {
                ui.Kp_jointControl,
                ui.cali_Kp_jointControl
        };
        QLineEdit * kis[2] = {
                ui.Kp_jointControl,
                ui.cali_Kp_jointControl
        };

        for (uint i = 0; i < 1; i++) {
            // skip updating calibration joints if button is not pressed yet
            if (i == 1) {

                if (!shouldUpdateSyncCalibrationJoints) {
                    ROS_INFO_THROTTLE(1, shouldUpdateSyncCalibrationJoints ? "Update joints!!"
                                                                           : "Do not update joints :-( ");
                    continue;
                } else
                    ROS_INFO_THROTTLE(1, "ROS SHOULD START calibration joints");
            }
            for (uint joint = 0; joint < NUMBER_OF_JOINT_SENSORS; joint++) {

                static float error_previous[NUMBER_OF_JOINT_SENSORS] = {0.0f, 0.0f, 0.0f, 0.0f};

                auto joint0 = joins[i][0];
                auto joint1 = joins[i][1];
                auto joint2 = joins[i][2];
                auto joint3 = joins[i][3];

                auto Ki_jointControl = kis[i];
                auto Kd_jointControl = kds[i];
                auto Kp_jointControl = kps[i];


                switch (joint) {
                    case 0:
                        error[joint] = joint0->value() - jointAngles[joint];
                        break;
                    case 1:
                        error[joint] = joint1->value() - jointAngles[joint];
                        break;
                    case 2:
                        error[joint] = joint2->value() - jointAngles[joint];
                        break;
                    case 3:
                        error[joint] = joint3->value() - jointAngles[joint];
                        break;
                }
                switch (joint) {
                    case 0: {
                        float pterm = atoi(Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        float result = pterm + dterm + integral[joint];
                        if (result >= 0.0f) {
                            myoMaster->changeSetPoint(4, result + 20);
                            myoMaster->changeSetPoint(6, 5);
                        } else if (result < 0.0f) {
                            myoMaster->changeSetPoint(4, 5);
                            myoMaster->changeSetPoint(6, fabsf(result) + 20);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 1: {
                        float pterm = atoi(Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        float result = pterm + dterm + integral[joint];
                        if (result >= 0.0f) {
                            myoMaster->changeSetPoint(1, result + 20);
                            myoMaster->changeSetPoint(3, 5);
                        } else if (result < 0.0f) {
                            myoMaster->changeSetPoint(1, 5);
                            myoMaster->changeSetPoint(3, fabsf(result) + 20);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 2: {
                        float pterm = atoi(Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] +=
                                (float) atoi(Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        float result = pterm + dterm + integral[joint];
                        ROS_INFO_THROTTLE(1, "\npterm %f\ndterm %f\niterm %f\nresult %f", pterm, dterm, integral[joint],
                                          result);
                        if (result >= 0.0f) {
                            myoMaster->changeSetPoint(12, result + 20);
                            myoMaster->changeSetPoint(10, 20);
                        } else if (result < 0.0f) {
                            myoMaster->changeSetPoint(12, 20);
                            myoMaster->changeSetPoint(10, fabsf(result) + 20);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                    case 3: {
                        float pterm = atoi(Kp_jointControl->text().toStdString().c_str()) * error[joint];
                        float dterm = atoi(Kd_jointControl->text().toStdString().c_str()) *
                                      (error[joint] - error_previous[joint]);
                        integral[joint] += atoi(Ki_jointControl->text().toStdString().c_str()) * error[joint];
                        if (integral[joint] >= integral_max) {
                            integral[joint] = integral_max;
                        } else if (integral[joint] <= integral_max) {
                            integral[joint] = -integral_max;
                        }
                        float result = pterm + dterm + integral[joint];
                        if (result >= 0.0f) {
                            myoMaster->changeSetPoint(13, result + 20);
                            myoMaster->changeSetPoint(8, 5);
                        } else if (result < 0.0f) {
                            myoMaster->changeSetPoint(13, 5);
                            myoMaster->changeSetPoint(8, fabsf(result) + 20);
                        }
                        error_previous[joint] = error[joint];
                        break;
                    }
                }
            }
            // joints has been updated and update flag must therefore be deactivated again to wait for the next button click
            if (i == 1) {
                shouldUpdateSyncCalibrationJoints = false;
            }
        }
    }
    static int counter = 0;
    if((counter++)%3==0)
        Q_EMIT newData(JOINT);
}

void MainWindow::MotorRecordPack(const roboy_communication_middleware::MotorRecord::ConstPtr &msg){
    numberOfRecordsToWaitFor--;
    records[msg->id][0] = msg->motor0;
    records[msg->id][1] = msg->motor1;
    records[msg->id][2] = msg->motor2;
    records[msg->id][3] = msg->motor3;
    records[msg->id][4] = msg->motor4;
    records[msg->id][5] = msg->motor5;
    records[msg->id][6] = msg->motor6;
    records[msg->id][7] = msg->motor7;
    records[msg->id][8] = msg->motor8;
    records[msg->id][9] = msg->motor9;
    records[msg->id][10] = msg->motor10;
    records[msg->id][11] = msg->motor11;
    records[msg->id][12] = msg->motor12;
    records[msg->id][13] = msg->motor13;
    ROS_INFO("received record from %d of length %ld with average sampling time %f ms",
    msg->id, msg->motor0.size(), msg->recordTime/msg->motor0.size()*1000.0f);
    if(numberOfRecordsToWaitFor==0){
        ROS_INFO("all records received");
        for(auto const& rec:records) {
            std::ofstream outfile;
            outfile.open(ui.movementName->text().toStdString().c_str());
            if (outfile.is_open()) {
                outfile << "<?xml version=\"1.0\" ?>"
                        << std::endl;
                outfile << "<roboy_trajectory control_mode=\""
                        << ui.control_mode->text().toStdString() << "\" samplingTime=\""
                        << atoi(ui.samplingTime->text().toStdString().c_str()) << "\">" << endl;
                for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
                    outfile << "<trajectory motorid=\"" << motor << "\">"
                            << std::endl;
                    for (uint i = 0; i < rec.second[motor].size(); i++)
                        outfile << rec.second[motor][i] << " ";
                    outfile << "</trajectory>" << std::endl;
                }
                outfile << "</roboy_trajectory>" << endl;
                outfile.close();
            }
        }
    }
}

void MainWindow::submitWaitJoints() {
    ignoreJointCommands = true;
}

void MainWindow::submitRunJoints() {
    ignoreJointCommands = false;
}

void MainWindow::plotData(int type) {
    switch(type) {
        case MOTOR:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot->graph(motor)->setData(time, motorData[0][motor][0]);
                ui.velocity_plot->graph(motor)->setData(time, motorData[0][motor][1]);
                ui.displacement_plot->graph(motor)->setData(time, motorData[0][motor][2]);
                ui.current_plot->graph(motor)->setData(time, motorData[0][motor][3]);

                if (motor == 0) {
                    ui.position_plot->graph(motor)->rescaleAxes();
                    ui.velocity_plot->graph(motor)->rescaleAxes();
                    ui.displacement_plot->graph(motor)->rescaleAxes();
                    ui.current_plot->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot->graph(motor)->rescaleAxes(true);
                    ui.current_plot->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot->replot();
            ui.velocity_plot->replot();
            ui.displacement_plot->replot();
            ui.current_plot->replot();
            break;
        case JOINT:
            ui.relAngle0_plot->graph(0)->setData(time, jointData[0][0][1]);
            ui.relAngle0_plot->xAxis->rescale();
            ui.relAngle0_plot->replot();

            ui.relAngle1_plot->graph(0)->setData(time, jointData[0][1][1]);
            ui.relAngle1_plot->xAxis->rescale();
            ui.relAngle1_plot->replot();

            ui.relAngle2_plot->graph(0)->setData(time, jointData[0][2][1]);
            ui.relAngle2_plot->xAxis->rescale();
            ui.relAngle2_plot->replot();

            ui.relAngle3_plot->graph(0)->setData(time, jointData[0][3][1]);
            ui.relAngle3_plot->xAxis->rescale();
            ui.relAngle3_plot->replot();
            break;
    }

}

bool MainWindow::playMovement(){
    // initialize TiXmlDocument doc with a string
    QModelIndexList indexList = ui.movementFolder->selectionModel()->selectedIndexes();
    ROS_INFO_STREAM("loading trajectory " << model->filePath(indexList[0]).toStdString().c_str());
    TiXmlDocument doc(model->filePath(indexList[0]).toStdString().c_str());
    if (!doc.LoadFile()) {
        return false;
    }

    TiXmlElement *root = doc.RootElement();

    int numberOfSamples, samplingTime, control_mode;

    root->QueryIntAttribute("control_mode", &control_mode);
    root->QueryIntAttribute("samplingTime",&samplingTime);
    ROS_INFO("recognized roboy_trajectory in control_mode %d with samplingTime %d", control_mode, samplingTime);

    roboy_communication_middleware::MotorRecord msg;

    // Constructs the myoMuscles by parsing custom xml.
    TiXmlElement *trajectory_it = NULL;
    for (trajectory_it = root->FirstChildElement("trajectory"); trajectory_it;
         trajectory_it = trajectory_it->NextSiblingElement("trajectory")) {
        int motor;
        if (trajectory_it->QueryIntAttribute("motorid", &motor)==TIXML_SUCCESS) {

            stringstream stream(trajectory_it->GetText());
            while (1) {
                int n;
                stream >> n;
                switch (motor) {
                    case 0:
                        msg.motor0.push_back(n);
                        break;
                    case 1:
                        msg.motor1.push_back(n);
                        break;
                    case 2:
                        msg.motor2.push_back(n);
                        break;
                    case 3:
                        msg.motor3.push_back(n);
                        break;
                    case 4:
                        msg.motor4.push_back(n);
                        break;
                    case 5:
                        msg.motor5.push_back(n);
                        break;
                    case 6:
                        msg.motor6.push_back(n);
                        break;
                    case 7:
                        msg.motor7.push_back(n);
                        break;
                    case 8:
                        msg.motor8.push_back(n);
                        break;
                    case 9:
                        msg.motor9.push_back(n);
                        break;
                    case 10:
                        msg.motor10.push_back(n);
                        break;
                    case 11:
                        msg.motor11.push_back(n);
                        break;
                    case 12:
                        msg.motor12.push_back(n);
                        break;
                    case 13:
                        msg.motor13.push_back(n);
                        break;
                    default:
                        ROS_ERROR("motorid %d is not available, aborting. check you recorded trajectory", motor);
                        return false;
                }

                if (!stream) {
                    numberOfSamples = msg.motor0.size();
                    break;
                }
            }
        }else{
            ROS_ERROR("trajectory without motorid, aborting. check you trajectory file");
            return false;
        }
    }

    msg.samplingTime = samplingTime;
    msg.control_mode = control_mode;

    motorTrajectory.publish(msg);
    return true;
}

void MainWindow::stopMovement(){
    ROS_INFO("stop movement");
    roboy_communication_middleware::MotorTrajectoryControl msg;
    msg.play = false;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::rewindMovement(){
    ROS_INFO("rewind movement");
    roboy_communication_middleware::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = true;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::pauseMovement(){
    ROS_INFO("pause movement");
    roboy_communication_middleware::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::loopMovement(){
    ROS_INFO("loop movement");
    roboy_communication_middleware::MotorTrajectoryControl msg;
    msg.play = true;
    msg.pause = ui.pause->isChecked();
    msg.rewind = false;
    msg.loop = ui.loop->isChecked();
    motorTrajectoryControl.publish(msg);
}

void MainWindow::submitCalibrationJoints(){
    ROS_INFO_THROTTLE(1, "Submit calibration clicked");
    shouldUpdateSyncCalibrationJoints = true;
    jointControl = true;
}

void MainWindow::stopButtonClicked(){
    ROS_INFO("stop button clicked");
    if(!ui.stop_button_motorControl->isChecked() && !ui.stop_button_jointControl->isChecked()) {
        updateControllerParams();
    }else{ // set controller gains to zero
        jointControl = false;
        motorControl = false;
        roboy_communication_middleware::MotorConfig msg;
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            msg.motors.push_back(motor);
            msg.control_mode.push_back(2);
            msg.outputPosMax.push_back(1000); // pwm max
            msg.outputNegMax.push_back(-1000); // pwm min
            msg.spPosMax.push_back(100000000);
            msg.spNegMax.push_back(-100000000);
            msg.IntegralPosMax.push_back(100);
            msg.IntegralNegMax.push_back(-100);
            msg.Kp.push_back(0);
            msg.Ki.push_back(0);
            msg.Kd.push_back(0);
            msg.forwardGain.push_back(0);
            msg.deadBand.push_back(0);
        }
        motorConfig.publish(msg);
    }
}

void MainWindow::updateSetPointsMotorControl(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    motorControl = true;
    jointControl = false;
    int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
    switch(ui.control_mode->value()){
        case POSITION:
            setpoints[0] = ui.motor0->value()*3000;
            setpoints[1] = ui.motor1->value()*3000;
            setpoints[2] = ui.motor2->value()*3000;
            setpoints[3] = ui.motor3->value()*3000;
            setpoints[4] = ui.motor4->value()*3000;
            setpoints[5] = ui.motor5->value()*3000;
            setpoints[6] = ui.motor6->value()*3000;
            setpoints[7] = ui.motor7->value()*3000;
            setpoints[8] = ui.motor8->value()*3000;
            setpoints[9] = ui.motor9->value()*3000;
            setpoints[10] = ui.motor10->value()*3000;
            setpoints[11] = ui.motor11->value()*3000;
            setpoints[12] = ui.motor12->value()*3000;
            setpoints[13] = ui.motor13->value()*3000;
            break;
        case VELOCITY:
            setpoints[0] = ui.motor0->value();
            setpoints[1] = ui.motor1->value();
            setpoints[2] = ui.motor2->value();
            setpoints[3] = ui.motor3->value();
            setpoints[4] = ui.motor4->value();
            setpoints[5] = ui.motor5->value();
            setpoints[6] = ui.motor6->value();
            setpoints[7] = ui.motor7->value();
            setpoints[8] = ui.motor8->value();
            setpoints[9] = ui.motor9->value();
            setpoints[10] = ui.motor10->value();
            setpoints[11] = ui.motor11->value();
            setpoints[12] = ui.motor12->value();
            setpoints[13] = ui.motor13->value();
            break;
        case DISPLACEMENT:
            setpoints[0] = (ui.motor0->value()+50)*10;
            setpoints[1] = (ui.motor1->value()+50)*10;
            setpoints[2] = (ui.motor2->value()+50)*10;
            setpoints[3] = (ui.motor3->value()+50)*10;
            setpoints[4] = (ui.motor4->value()+50)*10;
            setpoints[5] = (ui.motor5->value()+50)*10;
            setpoints[6] = (ui.motor6->value()+50)*10;
            setpoints[7] = (ui.motor7->value()+50)*10;
            setpoints[8] = (ui.motor8->value()+50)*10;
            setpoints[9] = (ui.motor9->value()+50)*10;
            setpoints[10] = (ui.motor10->value()+50)*10;
            setpoints[11] = (ui.motor11->value()+50)*10;
            setpoints[12] = (ui.motor12->value()+50)*10;
            setpoints[13] = (ui.motor13->value()+50)*10;
            break;
    }
    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        myoMaster->changeSetPoint(motor,setpoints[motor]);
    }
    ui.setPoint_motor0->setText(QString::number(setpoints[0]));
    ui.setPoint_motor1->setText(QString::number(setpoints[1]));
    ui.setPoint_motor2->setText(QString::number(setpoints[2]));
    ui.setPoint_motor3->setText(QString::number(setpoints[3]));
    ui.setPoint_motor4->setText(QString::number(setpoints[4]));
    ui.setPoint_motor5->setText(QString::number(setpoints[5]));
    ui.setPoint_motor6->setText(QString::number(setpoints[6]));
    ui.setPoint_motor7->setText(QString::number(setpoints[7]));
    ui.setPoint_motor8->setText(QString::number(setpoints[8]));
    ui.setPoint_motor9->setText(QString::number(setpoints[9]));
    ui.setPoint_motor10->setText(QString::number(setpoints[10]));
    ui.setPoint_motor11->setText(QString::number(setpoints[11]));
    ui.setPoint_motor12->setText(QString::number(setpoints[12]));
    ui.setPoint_motor13->setText(QString::number(setpoints[13]));
}

void MainWindow::updateSetPointsMotorControlAll(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    jointControl = false;
    motorControl = true;
    int setpoints[NUMBER_OF_MOTORS_PER_FPGA];
    switch(ui.control_mode->value()){
        case POSITION:
            setpoints[0] = ui.allMotors->value()*3000;
            setpoints[1] = ui.allMotors->value()*3000;
            setpoints[2] = ui.allMotors->value()*3000;
            setpoints[3] = ui.allMotors->value()*3000;
            setpoints[4] = ui.allMotors->value()*3000;
            setpoints[5] = ui.allMotors->value()*3000;
            setpoints[6] = ui.allMotors->value()*3000;
            setpoints[7] = ui.allMotors->value()*3000;
            setpoints[8] = ui.allMotors->value()*3000;
            setpoints[9] = ui.allMotors->value()*3000;
            setpoints[10] = ui.allMotors->value()*3000;
            setpoints[11] = ui.allMotors->value()*3000;
            setpoints[12] = ui.allMotors->value()*3000;
            setpoints[13] = ui.allMotors->value()*3000;
            break;
        case VELOCITY:
            setpoints[0] = ui.allMotors->value();
            setpoints[1] = ui.allMotors->value();
            setpoints[2] = ui.allMotors->value();
            setpoints[3] = ui.allMotors->value();
            setpoints[4] = ui.allMotors->value();
            setpoints[5] = ui.allMotors->value();
            setpoints[6] = ui.allMotors->value();
            setpoints[7] = ui.allMotors->value();
            setpoints[8] = ui.allMotors->value();
            setpoints[9] = ui.allMotors->value();
            setpoints[10] = ui.allMotors->value();
            setpoints[11] = ui.allMotors->value();
            setpoints[12] = ui.allMotors->value();
            setpoints[13] = ui.allMotors->value();
            break;
        case DISPLACEMENT:
            setpoints[0] = (ui.allMotors->value()+50)*10;
            setpoints[1] = (ui.allMotors->value()+50)*10;
            setpoints[2] = (ui.allMotors->value()+50)*10;
            setpoints[3] = (ui.allMotors->value()+50)*10;
            setpoints[4] = (ui.allMotors->value()+50)*10;
            setpoints[5] = (ui.allMotors->value()+50)*10;
            setpoints[6] = (ui.allMotors->value()+50)*10;
            setpoints[7] = (ui.allMotors->value()+50)*10;
            setpoints[8] = (ui.allMotors->value()+50)*10;
            setpoints[9] = (ui.allMotors->value()+50)*10;
            setpoints[10] = (ui.allMotors->value()+50)*10;
            setpoints[11] = (ui.allMotors->value()+50)*10;
            setpoints[12] = (ui.allMotors->value()+50)*10;
            setpoints[13] = (ui.allMotors->value()+50)*10;
            break;
    }
    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        myoMaster->changeSetPoint(motor,setpoints[motor]);
    }
    ui.setPoint_motor0->setText(QString::number(setpoints[0]));
    ui.setPoint_motor1->setText(QString::number(setpoints[1]));
    ui.setPoint_motor2->setText(QString::number(setpoints[2]));
    ui.setPoint_motor3->setText(QString::number(setpoints[3]));
    ui.setPoint_motor4->setText(QString::number(setpoints[4]));
    ui.setPoint_motor5->setText(QString::number(setpoints[5]));
    ui.setPoint_motor6->setText(QString::number(setpoints[6]));
    ui.setPoint_motor7->setText(QString::number(setpoints[7]));
    ui.setPoint_motor8->setText(QString::number(setpoints[8]));
    ui.setPoint_motor9->setText(QString::number(setpoints[9]));
    ui.setPoint_motor10->setText(QString::number(setpoints[10]));
    ui.setPoint_motor11->setText(QString::number(setpoints[11]));
    ui.setPoint_motor12->setText(QString::number(setpoints[12]));
    ui.setPoint_motor13->setText(QString::number(setpoints[13]));
}

void MainWindow::updateSetPointsJointControl(int val){
    jointControl = true;
    int setpoints[NUMBER_OF_JOINT_SENSORS];

    setpoints[0] = ui.joint0->value();
    setpoints[1] = ui.joint1->value();
    setpoints[2] = ui.joint2->value();
    setpoints[3] = ui.joint3->value();

//    for(int motor=0;motor<NUMBER_OF_JOINT_SENSORS;motor++){
//        myoMaster->changeSetPoint(motor,setpoints[motor]);
//    }
    ui.setPoint_joint0->setText(QString::number(setpoints[0]));
    ui.setPoint_joint1->setText(QString::number(setpoints[1]));
    ui.setPoint_joint2->setText(QString::number(setpoints[2]));
    ui.setPoint_joint3->setText(QString::number(setpoints[3]));
}

void MainWindow::updateCalibrationSetPointsJointControl(int val){
    /*jointControl = true;
    int setpoints[NUMBER_OF_JOINT_SENSORS];

    setpoints[0] = ui.cali_joint0->value();
    setpoints[1] = ui.cali_joint1->value();
    setpoints[2] = ui.cali_joint2->value();
    setpoints[3] = ui.cali_joint3->value();

//    for(int motor=0;motor<NUMBER_OF_JOINT_SENSORS;motor++){
//        myoMaster->changeSetPoint(motor,setpoints[motor]);
//    }
    ui.cali_setPoint_joint0->setText(QString::number(setpoints[0]));
    ui.cali_setPoint_joint1->setText(QString::number(setpoints[1]));
    ui.cali_setPoint_joint2->setText(QString::number(setpoints[2]));
    ui.cali_setPoint_joint3->setText(QString::number(setpoints[3]));
     */
}

void MainWindow::updateSetPointsJointControlAll(int val){
    jointControl = true;
    int setpoints[NUMBER_OF_JOINT_SENSORS];

    setpoints[0] = ui.allJoints->value();
    setpoints[1] = ui.allJoints->value();
    setpoints[2] = ui.allJoints->value();
    setpoints[3] = ui.allJoints->value();

//    for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
//        myoMaster->changeSetPoint(motor,setpoints[motor]);
//    }
    ui.setPoint_joint0->setText(QString::number(setpoints[0]));
    ui.setPoint_joint1->setText(QString::number(setpoints[1]));
    ui.setPoint_joint2->setText(QString::number(setpoints[2]));
    ui.setPoint_joint3->setText(QString::number(setpoints[3]));
}

void MainWindow::updateControllerParams(){
    ui.stop_button_motorControl->setChecked(false);
    roboy_communication_middleware::MotorConfig msg;
    for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
        msg.motors.push_back(motor);
        msg.control_mode.push_back(ui.control_mode->value());
        msg.outputPosMax.push_back(1000); // pwm max
        msg.outputNegMax.push_back(-1000); // pwm min
        msg.spPosMax.push_back(100000000);
        msg.spNegMax.push_back(-100000000);
        msg.IntegralPosMax.push_back(100);
        msg.IntegralNegMax.push_back(-100);
        msg.Kp.push_back(atoi(ui.Kp->text().toStdString().c_str()));
        msg.Ki.push_back(atoi(ui.Ki->text().toStdString().c_str()));
        msg.Kd.push_back(atoi(ui.Kd->text().toStdString().c_str()));
        msg.forwardGain.push_back(atoi(ui.forwardGain->text().toStdString().c_str()));
        msg.deadBand.push_back(atoi(ui.deadBand->text().toStdString().c_str()));
    }
    motorConfig.publish(msg);
}

void MainWindow::movementPathChanged(){
    model->setRootPath(QDir::currentPath());
    ui.movementFolder->setModel(model);
}

void MainWindow::recordMovement(){
    ROS_INFO("start recording");
    roboy_communication_middleware::MotorRecordConfig msg;
    msg.ids = 0;
    numberOfRecordsToWaitFor = 0;
    if(ui.record_fpga0->isChecked()) {
        msg.ids |= (1 << 0);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga1->isChecked()){
        msg.ids |= (1<<1);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga2->isChecked()){
        msg.ids |= (1<<2);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga3->isChecked()){
        msg.ids |= (1<<3);
        numberOfRecordsToWaitFor++;
    }
    if(ui.record_fpga4->isChecked()){
        msg.ids |= (1<<4);
        numberOfRecordsToWaitFor++;
    }

    msg.control_mode = ui.control_mode->value();
    msg.samplingTime = atoi(ui.samplingTime->text().toStdString().c_str());
    msg.recordTime = atoi(ui.recordTime->text().toStdString().c_str());
    motorRecordConfig.publish(msg);
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
//    QSettings settings("Qt-Ros Package", "interface");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
}

void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "interface");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace interface


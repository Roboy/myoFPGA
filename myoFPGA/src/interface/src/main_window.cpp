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

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(this, SIGNAL(newData(int)), this, SLOT(plotData(int)));

    QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor6, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor7, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor8, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor9, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor10, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor11, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor12, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.motor13, SIGNAL(valueChanged(int)), this, SLOT(updateSetPoints(int)));
    QObject::connect(ui.allMotors, SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsAll(int)));
    QObject::connect(ui.updateController, SIGNAL(clicked()), this, SLOT(updateControllerParams()));
    QObject::connect(ui.record, SIGNAL(clicked()), this, SLOT(recordMovement()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "interface",
                  ros::init_options::NoSigintHandler |
                          ros::init_options::AnonymousName|
                          ros::init_options::NoRosout);
    }
    motorStatus = nh->subscribe("/roboy/MotorStatus", 1, &MainWindow::MotorStatus, this);
    motorConfig = nh->advertise<communication::MotorConfig>("/roboy/MotorConfig", 1);
    motorRecordConfig = nh->advertise<communication::MotorRecordConfig>("/roboy/MotorRecordConfig", 1);
    motorRecord = nh->subscribe("/roboy/MotorRecord", 100, &MainWindow::MotorRecordPack, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
        ui.position_plot0->addGraph();
        ui.position_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot0->addGraph();
        ui.velocity_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.displacement_plot0->addGraph();
        ui.displacement_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.current_plot0->addGraph();
        ui.current_plot0->graph(motor)->setPen(QPen(color_pallette[motor]));
    }
    ui.position_plot0->xAxis->setLabel("x");
    ui.position_plot0->yAxis->setLabel("ticks");
    ui.position_plot0->replot();

    ui.velocity_plot0->xAxis->setLabel("x");
    ui.velocity_plot0->yAxis->setLabel("ticks/s");
    ui.velocity_plot0->replot();

    ui.displacement_plot0->xAxis->setLabel("x");
    ui.displacement_plot0->yAxis->setLabel("ticks");
    ui.displacement_plot0->replot();

    ui.current_plot0->xAxis->setLabel("x");
    ui.current_plot0->yAxis->setLabel("mA");
    ui.current_plot0->replot();

    updateControllerParams();

    model = new QFileSystemModel;
    movementPathChanged();
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

void MainWindow::MotorStatus(const communication::MotorStatus::ConstPtr &msg){
    ROS_INFO_THROTTLE(5, "receiving motor status");
    time.push_back(counter++);
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        motorData[msg->id][motor][0].push_back(msg->position[motor]);
        motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
        motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
        motorData[msg->id][motor][3].push_back(msg->current[motor]);
        if(motorData[msg->id][motor][0].size()>samples_per_plot){
            motorData[msg->id][motor][0].pop_front();
            motorData[msg->id][motor][1].pop_front();
            motorData[msg->id][motor][2].pop_front();
            motorData[msg->id][motor][3].pop_front();
        }
    }
    if(time.size()>samples_per_plot)
        time.pop_front();
    Q_EMIT newData(msg->id);
}

void MainWindow::MotorRecordPack(const communication::MotorRecord::ConstPtr &msg){
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
    ROS_INFO("received record from %d of length %d with average sampling time %f ms",
    msg->id, msg->motor0.size(), msg->recordTime/msg->motor0.size()*1000.0f);
    if(numberOfRecordsToWaitFor==0){
        ROS_INFO("all records received");
        for(auto const& rec:records) {
            std::ofstream outfile;
            outfile.open(ui.movementName->text().toStdString().c_str());
            if (outfile.is_open()) {
                outfile << "<?xml version=\"1.0\" ?>"
                        << std::endl;
                for(uint motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++) {
                    outfile << "<trajectory motorid=\"" << motor << "\" controlmode=\""
                            << ui.control_mode->text().toStdString() << "\" samplingTime=\""
                            << atoi(ui.samplingTime->text().toStdString().c_str()) << "\">"
                            << std::endl;
                    outfile << "<waypointlist>" << std::endl;
                    for (uint i = 0; i < rec.second[motor].size(); i++)
                        outfile << rec.second[motor][i] << " ";
                    outfile << "</waypointlist>" << std::endl;
                    outfile << "</trajectory>" << std::endl;
                }

                outfile << "</roboybehavior>" << std::endl;
                outfile.close();
            }

        }
    }
}

void MainWindow::plotData(int id) {
    switch(id){
        case 0:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot0->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot0->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot0->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot0->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot0->graph(motor)->rescaleAxes();
                    ui.velocity_plot0->graph(motor)->rescaleAxes();
                    ui.displacement_plot0->graph(motor)->rescaleAxes();
                    ui.current_plot0->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot0->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot0->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot0->graph(motor)->rescaleAxes(true);
                    ui.current_plot0->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot0->replot();
            ui.velocity_plot0->replot();
            ui.displacement_plot0->replot();
            ui.current_plot0->replot();
            break;
        case 1:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot1->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot1->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot1->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot1->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot1->graph(motor)->rescaleAxes();
                    ui.velocity_plot1->graph(motor)->rescaleAxes();
                    ui.displacement_plot1->graph(motor)->rescaleAxes();
                    ui.current_plot1->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot1->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot1->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot1->graph(motor)->rescaleAxes(true);
                    ui.current_plot1->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot1->replot();
            ui.velocity_plot1->replot();
            ui.displacement_plot1->replot();
            ui.current_plot1->replot();
            break;
        case 2:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot2->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot2->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot2->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot2->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot2->graph(motor)->rescaleAxes();
                    ui.velocity_plot2->graph(motor)->rescaleAxes();
                    ui.displacement_plot2->graph(motor)->rescaleAxes();
                    ui.current_plot2->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot2->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot2->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot2->graph(motor)->rescaleAxes(true);
                    ui.current_plot2->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot2->replot();
            ui.velocity_plot2->replot();
            ui.displacement_plot2->replot();
            ui.current_plot2->replot();
            break;
        case 3:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot3->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot3->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot3->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot3->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot3->graph(motor)->rescaleAxes();
                    ui.velocity_plot3->graph(motor)->rescaleAxes();
                    ui.displacement_plot3->graph(motor)->rescaleAxes();
                    ui.current_plot3->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot3->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot3->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot3->graph(motor)->rescaleAxes(true);
                    ui.current_plot3->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot3->replot();
            ui.velocity_plot3->replot();
            ui.displacement_plot3->replot();
            ui.current_plot3->replot();
            break;
        case 4:
            for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                ui.position_plot4->graph(motor)->setData(time, motorData[id][motor][0]);
                ui.velocity_plot4->graph(motor)->setData(time, motorData[id][motor][1]);
                ui.displacement_plot4->graph(motor)->setData(time, motorData[id][motor][2]);
                ui.current_plot4->graph(motor)->setData(time, motorData[id][motor][3]);

                if (motor == 0) {
                    ui.position_plot4->graph(motor)->rescaleAxes();
                    ui.velocity_plot4->graph(motor)->rescaleAxes();
                    ui.displacement_plot4->graph(motor)->rescaleAxes();
                    ui.current_plot4->graph(motor)->rescaleAxes();
                } else {
                    ui.position_plot4->graph(motor)->rescaleAxes(true);
                    ui.velocity_plot4->graph(motor)->rescaleAxes(true);
                    ui.displacement_plot4->graph(motor)->rescaleAxes(true);
                    ui.current_plot4->graph(motor)->rescaleAxes(true);
                }
            }
            ui.position_plot4->replot();
            ui.velocity_plot4->replot();
            ui.displacement_plot4->replot();
            ui.current_plot4->replot();
            break;
    }

}

void MainWindow::updateSetPoints(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    switch(ui.control_mode->value()){
        case POSITION:
            myoMaster->changeSetPoint(0,ui.motor0->value()*100000);
            myoMaster->changeSetPoint(1,ui.motor1->value()*100000);
            myoMaster->changeSetPoint(2,ui.motor2->value()*100000);
            myoMaster->changeSetPoint(3,ui.motor3->value()*100000);
            myoMaster->changeSetPoint(4,ui.motor4->value()*100000);
            myoMaster->changeSetPoint(5,ui.motor5->value()*100000);
            myoMaster->changeSetPoint(6,ui.motor6->value()*100000);
            myoMaster->changeSetPoint(7,ui.motor7->value()*100000);
            myoMaster->changeSetPoint(8,ui.motor8->value()*100000);
            myoMaster->changeSetPoint(9,ui.motor9->value()*100000);
            myoMaster->changeSetPoint(10,ui.motor10->value()*100000);
            myoMaster->changeSetPoint(11,ui.motor11->value()*100000);
            myoMaster->changeSetPoint(12,ui.motor12->value()*100000);
            myoMaster->changeSetPoint(13,ui.motor13->value()*100000);
            break;
        case VELOCITY:
            myoMaster->changeSetPoint(0,ui.motor0->value());
            myoMaster->changeSetPoint(1,ui.motor1->value());
            myoMaster->changeSetPoint(2,ui.motor2->value());
            myoMaster->changeSetPoint(3,ui.motor3->value());
            myoMaster->changeSetPoint(4,ui.motor4->value());
            myoMaster->changeSetPoint(5,ui.motor5->value());
            myoMaster->changeSetPoint(6,ui.motor6->value());
            myoMaster->changeSetPoint(7,ui.motor7->value());
            myoMaster->changeSetPoint(8,ui.motor8->value());
            myoMaster->changeSetPoint(9,ui.motor9->value());
            myoMaster->changeSetPoint(10,ui.motor10->value());
            myoMaster->changeSetPoint(11,ui.motor11->value());
            myoMaster->changeSetPoint(12,ui.motor12->value());
            myoMaster->changeSetPoint(13,ui.motor13->value());
            break;
        case DISPLACEMENT:
//            myoMaster->changeSetPoint(ui.motor0->value()*20);
            myoMaster->changeSetPoint(0,ui.motor0->value()*20);
            myoMaster->changeSetPoint(1,ui.motor1->value()*20);
            myoMaster->changeSetPoint(2,ui.motor2->value()*20);
            myoMaster->changeSetPoint(3,ui.motor3->value()*20);
            myoMaster->changeSetPoint(4,ui.motor4->value()*20);
            myoMaster->changeSetPoint(5,ui.motor5->value()*20);
            myoMaster->changeSetPoint(6,ui.motor6->value()*20);
            myoMaster->changeSetPoint(7,ui.motor7->value()*20);
            myoMaster->changeSetPoint(8,ui.motor8->value()*20);
            myoMaster->changeSetPoint(9,ui.motor9->value()*20);
            myoMaster->changeSetPoint(10,ui.motor10->value()*20);
            myoMaster->changeSetPoint(11,ui.motor11->value()*20);
            myoMaster->changeSetPoint(12,ui.motor12->value()*20);
            myoMaster->changeSetPoint(13,ui.motor13->value()*20);
            break;
    }
}

void MainWindow::updateSetPointsAll(int percent){
    std::lock_guard<std::mutex> lock(myoMaster->mux);
    switch(ui.control_mode->value()){
        case POSITION:
            myoMaster->changeSetPoint(0,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(1,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(2,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(3,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(4,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(5,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(6,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(7,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(8,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(9,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(10,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(11,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(12,ui.allMotors->value()*100000);
            myoMaster->changeSetPoint(13,ui.allMotors->value()*100000);
            break;
        case VELOCITY:
            myoMaster->changeSetPoint(0,ui.allMotors->value());
            myoMaster->changeSetPoint(1,ui.allMotors->value());
            myoMaster->changeSetPoint(2,ui.allMotors->value());
            myoMaster->changeSetPoint(3,ui.allMotors->value());
            myoMaster->changeSetPoint(4,ui.allMotors->value());
            myoMaster->changeSetPoint(5,ui.allMotors->value());
            myoMaster->changeSetPoint(6,ui.allMotors->value());
            myoMaster->changeSetPoint(7,ui.allMotors->value());
            myoMaster->changeSetPoint(8,ui.allMotors->value());
            myoMaster->changeSetPoint(9,ui.allMotors->value());
            myoMaster->changeSetPoint(10,ui.allMotors->value());
            myoMaster->changeSetPoint(11,ui.allMotors->value());
            myoMaster->changeSetPoint(12,ui.allMotors->value());
            myoMaster->changeSetPoint(13,ui.allMotors->value());
            break;
        case DISPLACEMENT:
            myoMaster->changeSetPoint(0,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(1,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(2,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(3,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(4,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(5,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(6,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(7,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(8,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(9,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(10,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(11,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(12,ui.allMotors->value()*20);
            myoMaster->changeSetPoint(13,ui.allMotors->value()*20);
            break;
    }
}

void MainWindow::updateControllerParams(){
    communication::MotorConfig msg;
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
    communication::MotorRecordConfig msg;
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


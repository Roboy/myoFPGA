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

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

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

    QObject::connect(ui.updateController, SIGNAL(clicked()), this, SLOT(updateControllerParams()));

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

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    ui.position_plot->addGraph();
    ui.position_plot->xAxis->setLabel("x");
    ui.position_plot->yAxis->setLabel("ticks");
    ui.position_plot->replot();

    ui.velocity_plot->addGraph();
    ui.velocity_plot->xAxis->setLabel("x");
    ui.velocity_plot->yAxis->setLabel("ticks/s");
    ui.velocity_plot->replot();

    ui.displacement_plot->addGraph();
    ui.displacement_plot->xAxis->setLabel("x");
    ui.displacement_plot->yAxis->setLabel("ticks");
    ui.displacement_plot->replot();

    ui.current_plot->addGraph();
    ui.current_plot->xAxis->setLabel("x");
    ui.current_plot->yAxis->setLabel("mA");
    ui.current_plot->replot();

    updateControllerParams();
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
    for (uint motor = 0; motor < 14; motor++) {
        motorData[motor][0].push_back(msg->position[motor]);
        motorData[motor][1].push_back(msg->velocity[motor]);
        motorData[motor][2].push_back(msg->displacement[motor]);
        motorData[motor][3].push_back(msg->current[motor]);
        if(motorData[motor][0].size()>samples_per_plot){
            motorData[motor][0].pop_front();
            motorData[motor][1].pop_front();
            motorData[motor][2].pop_front();
            motorData[motor][3].pop_front();
        }
    }
    if(time.size()>samples_per_plot)
        time.pop_front();
    Q_EMIT newData();
}

void MainWindow::plotData() {
    ui.position_plot->graph(0)->setData(time, motorData[0][0]);
    ui.position_plot->graph(0)->rescaleAxes();
    ui.position_plot->replot();

    ui.velocity_plot->graph(0)->setData(time, motorData[0][1]);
    ui.velocity_plot->graph(0)->rescaleAxes();
    ui.velocity_plot->replot();

    ui.displacement_plot->graph(0)->setData(time, motorData[0][2]);
    ui.displacement_plot->graph(0)->rescaleAxes();
    ui.displacement_plot->replot();

    ui.current_plot->graph(0)->setData(time, motorData[0][3]);
    ui.current_plot->graph(0)->rescaleAxes();
    ui.current_plot->replot();
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

void MainWindow::updateControllerParams(){
    communication::MotorConfig msg;
    for(uint motor=0;motor<14;motor++){
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

#include "interface.hpp"

//! standard query messages
char welcomestring[] = "commandline tool for controlling myode muscle via de0-nano setup";
char commandstring[] = "[0]position, [1]velocity, [2]force, [3]switch motor, [4]zero weight, [5]allToForce, [6]estimateSpringParams, [9]exit";
char setpointstring[] = "set point (rad) ?";
char setvelstring[] = "set velocity (rad/s) ?";
char setforcestring[] = "set force (N) ?";
char motorstring[] = "which motor(0-3)?";
char motorinfo[30];
char ganglionstring[] = "which ganglion(0-5)?";
char runningstring[] = "running ";
char recordingstring[] = "recording ";
char donestring[] = "done ";
char samplingtimestring[] = "samplingTime [milliseconds]: ";
char recordtimestring[] = "recordTime [seconds]: ";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char averageconnectionspeedstring[] = "average connection speed: ";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char remotecontrolactivestring[] = "remote control active [hit q to quit]";
char publishingmotorstring[] = "publishing motor status[hit q to quit]";
char receivedupdatestring[] = "received update";
char errormessage[] = "Error: received update for motor that is not connected";
char byebyestring[] = "BYE BYE!";

enum COLORS {
    CYAN = 1,
    RED,
    GREEN,
};

Interface::Interface(vector<int32_t*> &pid_base, int motors) {
	myoControl = new MyoControl(pid_base, motors);
	//! start ncurses mode
	initscr();
	//! Start color functionality
//        start_color();
	init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
	init_pair(RED, COLOR_RED, COLOR_BLACK);
	init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
	//! get the size of the terminal window
	getmaxyx(stdscr, rows, cols);

	print(0, 0, cols, "-");
	printMessage(1, 0, welcomestring);
	print(2, 0, cols, "-");
	print(6, 0, cols, "-");
	querySensoryData();
	printMessage(3, 0, commandstring);
}

Interface::~Interface() {
	delete myoControl;
	clearAll(0);
	printMessage(rows / 2, cols / 2 - strlen(byebyestring) / 2, byebyestring);
	refresh();
	usleep(1000000);
	endwin();
}

void Interface::printMessage(uint row, uint col, char *msg) {
	mvprintw(row, col, "%s", msg);
	refresh();
}

void Interface::printMessage(uint row, uint col, char *msg, uint color) {
	mvprintw(row, col, "%s", msg);
	mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
	refresh();
}

void Interface::print(uint row, uint startcol, uint length, const char *s) {
	for (uint i = startcol; i < startcol + length; i++) {
		mvprintw(row, i, "%s", s);
	}
	refresh();
}

void Interface::clearAll(uint row) {
	for (uint i = row; i < rows; i++) {
		print(i, 0, cols, " ");
	}
	refresh();
}

void Interface::querySensoryData() {
	myoControl->update();
	motor.actuatorPos = myoControl->pos[motor_id];
	motor.actuatorVel = myoControl->vel[motor_id];
	motor.actuatorCurrent = myoControl->current[motor_id];
	motor.tendonDisplacement = myoControl->displacement[motor_id];

	sprintf(motorinfo, "motor %d   ", motor_id);
	printMessage(7, 0, motorinfo, CYAN);
	mvprintw(8, 0, "pwm:                 %d      ", myoControl->pwm_control[motor_id]);
	mvprintw(9, 0, "actuatorPos (rad):   %d    ", motor.actuatorPos);
	mvprintw(10, 0, "actuatorVel (rad/s): %d   ", motor.actuatorVel);
	mvprintw(11, 0, "actuatorCurrent:     %d     ", motor.actuatorCurrent);
	mvprintw(12, 0, "tendonDisplacement:  %d     ", motor.tendonDisplacement);
	print(13, 0, cols, "-");
	float Pgain, Igain, Dgain, forwardGain, deadband, setPoint, setPointMin, setPointMax;
	switch(myoControl->control_mode[motor_id]){
	case Position:
		Pgain = myoControl->control_params[Position][motor_id].params.pidParameters.pgain;
		Igain = myoControl->control_params[Position][motor_id].params.pidParameters.igain;
		Dgain = myoControl->control_params[Position][motor_id].params.pidParameters.dgain;
		forwardGain = myoControl->control_params[Position][motor_id].params.pidParameters.forwardGain;
		deadband = myoControl->control_params[Position][motor_id].params.pidParameters.deadBand;
		setPoint = myoControl->pos_setPoint[motor_id];
		setPointMin = myoControl->control_params[Position][motor_id].spNegMax;
		setPointMax = myoControl->control_params[Position][motor_id].spPosMax;
		break;
	case Velocity:
		Pgain = myoControl->control_params[Velocity][motor_id].params.pidParameters.pgain;
		Igain = myoControl->control_params[Velocity][motor_id].params.pidParameters.igain;
		Dgain = myoControl->control_params[Velocity][motor_id].params.pidParameters.dgain;
		forwardGain = myoControl->control_params[Velocity][motor_id].params.pidParameters.forwardGain;
		deadband = myoControl->control_params[Velocity][motor_id].params.pidParameters.deadBand;
		setPoint = myoControl->pos_setPoint[motor_id];
		setPointMin = myoControl->control_params[Velocity][motor_id].spNegMax;
		setPointMax = myoControl->control_params[Velocity][motor_id].spPosMax;
		break;
	case Force:
		Pgain = myoControl->control_params[Force][motor_id].params.pidParameters.pgain;
		Igain = myoControl->control_params[Force][motor_id].params.pidParameters.igain;
		Dgain = myoControl->control_params[Force][motor_id].params.pidParameters.dgain;
		forwardGain = myoControl->control_params[Force][motor_id].params.pidParameters.forwardGain;
		deadband = myoControl->control_params[Force][motor_id].params.pidParameters.deadBand;
		setPoint = myoControl->pos_setPoint[motor_id];
		setPointMin = myoControl->control_params[Force][motor_id].spNegMax;
		setPointMax = myoControl->control_params[Force][motor_id].spPosMax;
		break;
	default:
		break;
	}
	mvprintw(14, 0, "P gain:          %.5f       ", Pgain);
	mvprintw(15, 0, "I gain:          %.5f       ", Igain);
	mvprintw(16, 0, "D gain:          %.5f       ", Dgain);
	mvprintw(17, 0, "forward gain:    %.5f       ", forwardGain);
	mvprintw(18, 0, "deadband:        %.5f       ", deadband);
	mvprintw(19, 0, "set point:       %.5f       ", setPoint);
	print(20, 0, cols, "-");
	mvprintw(21, 0, "polyPar: %.5f  %.5f  %.5f  %.5f    ", myoControl->polyPar[motor_id][0],
			myoControl->polyPar[motor_id][1], myoControl->polyPar[motor_id][2],
			myoControl->polyPar[motor_id][3]);
	mvprintw(22, 0, "set point limits: %.5f to %.5f     ", setPointMin, setPointMax);
	mvprintw(23, 0, "weight: %.2f     ", myoControl->getWeight());
	refresh();
}

void Interface::processing(char *msg1, char *what, char *msg2) {
	char cmd;
	uint a = strlen(msg1);
	uint b = strlen(what);
	uint c = strlen(msg2);

	print(5, 0, cols, " ");
	printMessage(5, 0, msg1);
	printMessage(5, a + 1, what);
	printMessage(5, a + 1 + b + 1, msg2);
	mvchgat(5, 0, a + 1 + b, A_BLINK, 2, NULL);
	mvchgat(5, a + 1 + b + 1, a + 1 + b + 1 + c, A_BLINK, 1, NULL);
	timeout(timeout_ms);
	do {
		querySensoryData();
		cmd = mvgetch(5, a + 1 + b + 1 + c);
	} while (cmd != 'q');
	timeout(-1);
}

void Interface::processing(char *msg1, char *msg2) {
	char cmd;
	uint a = strlen(msg1);
	uint c = strlen(msg2);

	print(5, 0, cols, " ");
	printMessage(5, 0, msg1);
	printMessage(5, a + 1, msg2);
	mvchgat(5, 0, a, A_BLINK, 2, NULL);
	mvchgat(5, a + 1, a + 1 + c, A_BLINK, 1, NULL);
	timeout(timeout_ms);
	do {
		querySensoryData();
		cmd = mvgetch(5, a + 1 + c);
	} while (cmd != 'q');
	timeout(-1);
}

void Interface::positionControl() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	myoControl->changeControl(motor_id, Position);
	printMessage(4, 0, setpointstring);
	mvchgat(4, 0, strlen(setpointstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(5, 0, inputstring, 30);
	pos = atof(inputstring);
	myoControl->setPosition(motor_id, pos);
	processing(runningstring, inputstring, quitstring);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::velocityControl() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	myoControl->changeControl(motor_id, Velocity);
	printMessage(4, 0, setvelstring);
	mvchgat(4, 0, strlen(setvelstring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(5, 0, inputstring, 30);
	pos = atof(inputstring);
	myoControl->setVelocity(motor_id, pos);
	processing(runningstring, inputstring, quitstring);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::forceControl() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	myoControl->changeControl(motor_id, Force);
	printMessage(4, 0, setforcestring);
	mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(5, 0, inputstring, 30);
	pos = atof(inputstring);
	myoControl->setForce(motor_id, pos);
	processing(runningstring, inputstring, quitstring);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::switchMotor() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, ganglionstring, GREEN);
	mvgetnstr(5, 0, inputstring, 30);
	uint ganlionrequest = atoi(inputstring);
	if (ganlionrequest < 6)
		ganglion_id = ganlionrequest;
	else {
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		printMessage(5, 0, invalidstring, RED);
		return;
	}
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, motorstring, GREEN);
	mvgetnstr(5, 0, inputstring, 30);
	uint motorrequest = atoi(inputstring);
	if (motorrequest < 4)
		motor_id = motorrequest;
	else {
		print(4, 0, cols, " ");
		print(5, 0, cols, " ");
		printMessage(5, 0, invalidstring, RED);
		return;
	}
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::zeroWeight(){
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, "zeroing weight");
	myoControl->zeroWeight();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
}

void Interface::setAllToForce() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, setforcestring);
	mvchgat(4, 0, strlen(setforcestring), A_BOLD, 1, NULL);
	refresh();
	mvgetnstr(5, 0, inputstring, 30);
	pos = atof(inputstring);
	myoControl->allToForce(pos);
	processing(runningstring, inputstring, quitstring);
	// set back to zero force
	myoControl->allToForce(0);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::estimateSpringParameters(){
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	printMessage(4, 0, "estimating spring paramters, please wait...");
	myoControl->estimateSpringParameters(motor_id, 600000, 1000);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
}

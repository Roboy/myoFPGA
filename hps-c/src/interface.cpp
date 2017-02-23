#include "interface.hpp"

//! standard query messages
char welcomestring[] = "commandline tool for controlling myode muscle via de0-nano setup";
char commandstring[] = "[0]position, [1]velocity, [2]force, [3]switch motor, [4]connection speed, [5]record, [6]allToForce, [7] resettingSpring, [8] resettAll, [r]android, [p]publishMotorInfo, [9]exit";
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

Interface::Interface(uint32_t *spi_base, int motors) {
	myoControl = new MyoControl(spi_base, motors);
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
	mvprintw(8, 0, "actuatorPos (rad):   %.5f    ", motor.actuatorPos);
	mvprintw(9, 0, "actuatorVel (rad/s): %.5f    ", motor.actuatorVel);
	mvprintw(10, 0, "actuatorCurrent:     %d     ", motor.actuatorCurrent);
	mvprintw(11, 0, "tendonDisplacement:  %.5f   ", (float) motor.tendonDisplacement / 32768.0f);
	print(12, 0, cols, "-");
	float Pgain, Igain, Dgain, forwardGain, deadband, setPoint, setPointMin, setPointMax;
	switch(myoControl->control_mode[motor_id]){
	case Position:
		Pgain = myoControl->position_controller[motor_id].pgain;
		Igain = myoControl->position_controller[motor_id].igain;
		Dgain = myoControl->position_controller[motor_id].dgain;
		forwardGain = myoControl->position_controller[motor_id].forwardGain;
		deadband = myoControl->position_controller[motor_id].deadBand;
		setPoint = myoControl->pos_setPoint[motor_id];
		setPointMin = myoControl->position_controller[motor_id].spNegMax;
		setPointMax = myoControl->position_controller[motor_id].spPosMax;
		break;
	case Velocity:
		Pgain = myoControl->velocity_controller[motor_id].pgain;
		Igain = myoControl->velocity_controller[motor_id].igain;
		Dgain = myoControl->velocity_controller[motor_id].dgain;
		forwardGain = myoControl->velocity_controller[motor_id].forwardGain;
		deadband = myoControl->velocity_controller[motor_id].deadBand;
		setPoint = myoControl->vel_setPoint[motor_id];
		setPointMin = myoControl->velocity_controller[motor_id].spNegMax;
		setPointMax = myoControl->velocity_controller[motor_id].spPosMax;
		break;
	case Force:
		Pgain = myoControl->force_controller[motor_id].pgain;
		Igain = myoControl->force_controller[motor_id].igain;
		Dgain = myoControl->force_controller[motor_id].dgain;
		forwardGain = myoControl->force_controller[motor_id].forwardGain;
		deadband = myoControl->force_controller[motor_id].deadBand;
		setPoint = myoControl->force_setPoint[motor_id];
		setPointMin = myoControl->force_controller[motor_id].spNegMax;
		setPointMax = myoControl->force_controller[motor_id].spPosMax;
		break;
	default:
		break;
	}
	mvprintw(13, 0, "P gain:          %.5f       ", Pgain);
	mvprintw(14, 0, "I gain:          %.5f       ", Igain);
	mvprintw(15, 0, "D gain:          %.5f       ", Dgain);
	mvprintw(16, 0, "forward gain:    %.5f       ", forwardGain);
	mvprintw(17, 0, "deadband:        %.5f       ", deadband);
	mvprintw(18, 0, "set point:       %.5f   ", setPoint);
	print(19, 0, cols, "-");
	mvprintw(20, 0, "polyPar: %.5f  %.5f  %.5f  %.5f    ", myoControl->polyPar[0], myoControl->polyPar[1], myoControl->polyPar[2], myoControl->polyPar[3]);
	mvprintw(21, 0, "set point limits: %.5f to %.5f     ", setPointMin, setPointMax);
	if(myoControl->adc_base!=nullptr){
		*myoControl->adc_base = 0;
		mvprintw(22, 0, "weight: %.2f     ", 83.7f - 0.0455f*(*myoControl->adc_base));
	}
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
	timeout(10);
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
	timeout(10);
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

void Interface::measureConnection() {
	timeout(-1);
	echo();
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	double averageTime = 0;
	printMessage(4, 0, averageconnectionspeedstring);
	char str[20];
	sprintf(str, "%f seconds", averageTime);
	printMessage(4, strlen(averageconnectionspeedstring), str, CYAN);
	printMessage(5, 0, logfilestring, CYAN);
	usleep(5000000);
	print(4, 0, cols, " ");
	print(5, 0, cols, " ");
	noecho();
}

void Interface::recordTrajectories() {
//        timeout(-1);
//        echo();
//        print(4, 0, cols, " ");
//        print(5, 0, cols, " ");
//        printMessage(4, 0, filenamestring);
//        mvgetnstr(4, strlen(filenamestring), inputstring, 30);
//        std::string name(inputstring);
//        print(4, 0, cols, " ");
//        printMessage(4, 0, samplingtimestring, CYAN);
//        mvgetnstr(4, strlen(samplingtimestring), inputstring, 30);
//        float samplingTime = atof(inputstring);
//        printMessage(5, 0, recordtimestring, CYAN);
//        mvgetnstr(5, strlen(recordtimestring), inputstring, 30);
//        double recordTime = atof(inputstring);
//        print(4, 0, cols, " ");
//        print(5, 0, cols, " ");
//        printMessage(4, 0, recordingstring, RED);
//        std::vector<std::vector<float>> trajectories;
//        std::vector<int> idList = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
//        std::vector<int> controlmode(16, 1);
//        float averageSamplingTime = 0;
//        print(4, 0, cols, " ");
//        printMessage(4, 0, donestring, GREEN);
//        char averagetimestring[50];
//        sprintf(averagetimestring, "average %s%f", samplingtimestring, averageSamplingTime);
//        printMessage(4, strlen(donestring), averagetimestring, CYAN);
//        usleep(500000);
//        print(4, 0, cols, " ");
//        print(5, 0, cols, " ");
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

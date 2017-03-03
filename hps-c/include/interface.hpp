#pragma once

#include <ncurses.h>
#include <vector>
#include <cstring>
#include "myoControl.hpp"

using namespace std;

class Interface {
public:
    Interface(uint32_t *spi_base, vector<int32_t*> &pid_base, int motors);

    ~Interface();

    void printMessage(uint row, uint col, char *msg);

    void printMessage(uint row, uint col, char *msg, uint color);

    void print(uint row, uint startcol, uint length, const char *s);

    void clearAll(uint row);

    void querySensoryData();

    void processing(char *msg1, char *what, char *msg2);

    void processing(char *msg1, char *msg2);

    void positionControl();

    void velocityControl();

    void forceControl();

    void switchMotor();

    void zeroWeight();

    void setAllToForce();

    void estimateSpringParameters();

    MyoControl *myoControl;
    uint timeout_ms = 10;
private:
    uint rows, cols;
    float pos;
    uint ganglion_id = 0;
    uint motor_id = 0;
    char inputstring[30];

    struct MotorData {
            float jointPos;
            float actuatorPos;
            float actuatorVel;
            uint16 actuatorCurrent;
            sint16 tendonDisplacement;
        } motor;

};

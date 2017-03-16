#pragma once

#include <ncurses.h>
#include <vector>
#include <cstring>
#include <unistd.h>
#include "myoControl.hpp"

using namespace std;

class Interface {
public:
    Interface(vector<int32_t*> &myo_base);

    ~Interface();

    void printMessage(uint row, uint col, char *msg);

    void printMessage(uint row, uint col, char *msg, uint color);

    void print(uint row, uint startcol, uint length, const char *s);

    void clearAll(uint row);

    void querySensoryData();

    void processing(char *msg1, char *what, char *msg2);

    void processing(char *msg1, char *msg2);

    void toggleSPI();

    void reset();

    void positionControl();

    void velocityControl();

    void displacementControl();

    void switchMotor();

    void zeroWeight();

    void setAllToForce();

    void estimateSpringParameters();

    MyoControl *myoControl;
    uint timeout_ms = 10;
private:
    uint rows, cols;
    int pos;
    uint ganglion_id = 0;
    uint motor_id = 0;
    char inputstring[30];

    struct MotorData {
            int32_t jointPos;
            int32_t actuatorPos;
            int32_t actuatorVel;
            uint16 actuatorCurrent;
            sint16 tendonDisplacement;
        } motor;

};

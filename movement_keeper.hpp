#ifndef MOVEMENTKEEPER
#define MOVEMENTKEEPER

#include <ros/ros.h>
#include <cmath>


class MovementKeeper {
    public:
        MovementKeeper();
        float getAngle();
        float getXPos();
        float getYPos();
        void makeMoves(float x, float ang);
        void reset();
        void initTime();
    private:
        float angle;
        float xpos;
        float ypos;
        float lastCmd;
};

MovementKeeper::MovementKeeper() {
    angle = 0;
    xpos = 0;
    ypos = 0;
}

void MovementKeeper::initTime() {
    lastCmd = ros::Time::now().toSec();
}

void MovementKeeper::reset() {
    angle = 0;
    xpos = 0;
    ypos = 0;
    lastCmd = ros::Time::now().toSec();
}

float MovementKeeper::getAngle() {
    return angle;
}

float MovementKeeper::getXPos() {
    return xpos;
}

float MovementKeeper::getYPos() {
    return ypos;
}

void MovementKeeper::makeMoves(float x, float ang) {
    float t = ros::Time::now().toSec() - lastCmd;

    angle += t*ang;
    xpos += t*x*cos(angle);
    ypos += t*x*sin(angle);

    lastCmd = ros::Time::now().toSec();
}

// ---
// linear: 
//   x: 0.5
//   y: 0.0
//   z: 0.0
// angular: 
//   x: 0.0
//   y: 0.0
//   z: 0.168472617865

#endif

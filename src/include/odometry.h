#pragma once
#include "main.h"

#define centerToXTracking 2 // distance from tracking center to x tracking wheel
#define centerToYTracking 0 // distance from tracking center to y tracking wheel
#define wheelDiameter 3.25 // diameter of the side wheels
#define trackingDiameter 1.5 // diameter of tracking wheel

namespace odometry {
    struct Robot {
        double x;
        double y;
        double angle;
        bool team;
    };

    extern Robot robot;
    extern double targetAngle;
    extern double targetDistance;

    extern void init_odometry();
    extern void updateSensors();
    extern void updatePosition();
    extern void positionTrack(void* param);
    extern void move_to_point(double x, double y, double driveSpeed, double turnSpeed);
    extern void move_to_point_no_turn(double x, double y, double driveSpeed, double turnSpeed);
}
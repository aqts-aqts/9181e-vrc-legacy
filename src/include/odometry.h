#pragma once
#include "main.h"

#define centerToXTracking 5.54753 // distance from tracking center to tracking wheel
#define centerToYTracking 0.17076 // distance from tracking center to middle of left wheels
#define wheelDiameter 3.25 // diameter of the side wheels
#define trackingDiameter 2.75 // diameter of tracking wheel

// #define blueHeight 0 // height of blue goal
// #define redHeight 0 // height of red goal
// #define launchAngle 0 // angle of flywheel launch
// #define launchHeight 0 // height of flywheel launch

// #define gravity 9.81 // gravitational constant
// #define flyRadius 0 // radius of flywheel in METERS

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
    extern void set_approach_constants(double p, double d, double i, double c, double psi, double csi);
    extern void set_approach_exit_conditions(double small_err, double small_time, double big_err, double big_time, double velocity_err, double velocity_time, double mA_time);
    extern void approach_to_point(double x, double y, double maxSpeed, double maxturnSpeed);
}
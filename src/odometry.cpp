#include "main.h"
using namespace global;

const int DRIVE_SPEED = 110;
const int TURN_SPEED  = 90;

double curXTrack = 0;
double curYTrack = 0;

double lastXTrack = 0;
double lastYTrack = 0;
double lastAngle = 0;

double deltaAngle = 0;
double deltaXTrack = 0;
double deltaYTrack = 0;

double deltaX = 0;
double deltaY = 0;

double error = 0;

namespace odometry {
    Robot robot; 
    double targetAngle;
    double targetDistance;

    void init_odometry() {
        robot.x = 0; // x coordinate of robot tracking center
        robot.y = 0; // y coordinate of robot tracking center
        robot.angle = 0; // total rotation of robot in radians
        robot.team = 0; // blue = 0, red = 1
    }

    void updateSensors() {
        lastXTrack = curXTrack; // x tracking wheel position at last update
        lastYTrack = curYTrack; // y tracking wheel position at last update
        lastAngle = robot.angle; // rotation of robot in radians at last update

        curXTrack = (horizontalEncoder.get_position() / 100.0) * trackingDiameter * M_PI / 360; // current x tracking wheel position
        curYTrack = (verticalEncoder.get_position() / 100.0) * trackingDiameter * M_PI / 360;  // current y tracking wheel position
        robot.angle = chassis.imu.get_rotation() * M_PI / 180; // current rotation of robot in radians
    }

    void updatePosition() {
        deltaXTrack = curXTrack - lastXTrack; // change in x tracking wheel position
        deltaYTrack = curYTrack - lastYTrack; // change in y tracking wheel position
        deltaAngle = robot.angle - lastAngle; // change in rotation of robot in radians

        if (deltaAngle == 0) { // if robot is not rotating
            deltaX = deltaXTrack; // set change in x position to change in x tracking wheel position
            deltaY = deltaYTrack; // set change in y position to change in y tracking wheel position
        } else {
            deltaX = 2 * sin(deltaAngle / 2) * (deltaXTrack / deltaAngle + centerToXTracking); // use trig to find change in x position
            deltaY = 2 * sin(deltaAngle / 2) * (deltaYTrack / deltaAngle + centerToYTracking); // use trig to find change in y position

        }

        robot.x += cos(lastAngle) * deltaX + sin(lastAngle) * deltaY; // convert change in x position to coordinates
        robot.y += -sin(lastAngle) * deltaX + cos(lastAngle) * deltaY; // convert change in y position to coordinates
    }

    void updateDisplay() {
        pros::lcd::print(0, "X: %f", odometry::robot.x);
        pros::lcd::print(1, "Y: %f", odometry::robot.y);
        pros::lcd::print(2, "Angle: %f", odometry::robot.angle);
    }

    void positionTrack(void* param) {
        while (true) {
            updateSensors();
            updatePosition();
            updateDisplay();
            pros::delay(10);
        }
    }

    void move_to_point(double x, double y, double maxSpeed = DRIVE_SPEED, double maxturnSpeed = TURN_SPEED) {
        double angle = fmod(chassis.imu.get_heading() + atan2(y - robot.y, x - robot.x) * 180 / M_PI, 360.0); // constrict angle

        chassis.set_turn_pid(angle, maxturnSpeed);
        chassis.wait_drive();

        error = fabs(sqrt(pow(x - robot.x, 2) + pow(y - robot.y, 2)));
        chassis.set_drive_pid(error, maxSpeed);
        chassis.wait_drive();
    }

    void move_to_point_no_turn(double x, double y, double maxSpeed = DRIVE_SPEED) {
        error = fabs(sqrt(pow(x - robot.x, 2) + pow(y - robot.y, 2)));
        chassis.set_drive_pid(error, maxSpeed);
        chassis.wait_drive();
    }
}
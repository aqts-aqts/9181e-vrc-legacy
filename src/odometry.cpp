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
        robot.x = 0;
        robot.y = 0;
        robot.angle = 0;
        robot.team = 0; // blue = 0, red = 1
    }

    void updateSensors() {
        lastXTrack = curXTrack;
        lastYTrack = curYTrack;
        lastAngle = robot.angle;

        curXTrack = (horizontalEncoder.get_position() / 100.0) * trackingDiameter * M_PI / 360;
        curYTrack = (verticalEncoder.get_position() / 100.0) * trackingDiameter * M_PI / 360; 
        robot.angle = chassis.imu.get_rotation() * M_PI / 180;
    }

    void updatePosition() {
        deltaXTrack = curXTrack - lastXTrack;
        deltaYTrack = curYTrack - lastYTrack;
        deltaAngle = robot.angle - lastAngle;

        if (deltaAngle == 0) {
            deltaX = deltaXTrack;
            deltaY = deltaYTrack;
        } else {
            deltaX = 2 * sin(deltaAngle / 2) * (deltaXTrack / deltaAngle + centerToXTracking);
            deltaY = 2 * sin(deltaAngle / 2) * (deltaYTrack / deltaAngle + centerToYTracking);
        }

        robot.x += cos(lastAngle) * deltaX + sin(lastAngle) * deltaY;
        robot.y += -sin(lastAngle) * deltaX + cos(lastAngle) * deltaY;
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
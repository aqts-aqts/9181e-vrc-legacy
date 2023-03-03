#include "main.h"
using namespace global;

double targetX;
double targetY;

double dx;
double dy;

namespace global {
    void aim() {
        targetX = (odometry::robot.team) ? blueGoalX: redGoalX; // get target goal position
        targetY = (odometry::robot.team) ? blueGoalY: redGoalY;

        dx = targetX - odometry::robot.x;
        dy = targetY - odometry::robot.y;
        
        double deltaTheta = atan2(dy, dx) * 180 / M_PI;
        double theta = chassis.imu.get_heading() + deltaTheta;
        
        chassis.set_turn_pid(theta, aimSpeed);
        chassis.wait_drive();
    }

    void roll(double rollerSpeed) {
        pros::c::optical_rgb_s_t RGB = colour.get_rgb();
        intake.move(rollerSpeed * reverseIntake);

        bool initColour = (RGB.red > RGB.blue); // blue = 0, red = 1

        if (odometry::robot.team && initColour) {
            while (RGB.red > RGB.blue) {
                RGB = colour.get_rgb();
                pros::delay(10);  
            }
            while (RGB.blue > RGB.red) {
                RGB = colour.get_rgb();
                pros::delay(10); 
            }
        } else if (odometry::robot.team && !initColour) {
            while (RGB.blue > RGB.red) {
                RGB = colour.get_rgb();
                pros::delay(10);  
            }
        } else if (!odometry::robot.team && initColour) {
            while (RGB.red > RGB.blue) {
                RGB = colour.get_rgb();
                pros::delay(10); 
            }
        } else {
            while (RGB.blue > RGB.red) {
                RGB = colour.get_rgb();
                pros::delay(10);  
            }
            while (RGB.red > RGB.blue) {
                RGB = colour.get_rgb();
                pros::delay(10); 
            }
        }

        intake.move(0);
    }

    void back(bool axis) { // 0 - X, 1 = Y
        double target = (axis) ? rollerX2: rollerY1;
        double error = target - ((axis) ? odometry::robot.x: odometry::robot.y);

        chassis.set_drive_pid(error, backSpeed);
        chassis.wait_drive();
    }
}
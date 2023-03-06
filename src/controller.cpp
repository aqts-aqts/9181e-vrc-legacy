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

        dx = targetX - odometry::robot.x; // get difference in position
        dy = targetY - odometry::robot.y;
        
        double deltaTheta = atan2(dy, dx) * 180 / M_PI; // get difference in angle
        double theta = fmod(chassis.imu.get_heading() + deltaTheta, 360); // get target angle
        
        chassis.set_turn_pid(theta, aimSpeed); // turn to angle
        chassis.wait_drive();
    }

    void roll(double rollerSpeed) {
        pros::c::optical_rgb_s_t RGB = colour.get_rgb(); // get colour sensor value
        intake.move(rollerSpeed * reverseIntake); // start intake to roll

        bool initColour = (RGB.red > RGB.blue); // get inital colour value (blue = 0, red = 1)
        int rollTime = 0; // total time rolled (used to prevent infinite rolling)

        if (odometry::robot.team && initColour) { // if team is red and inital colour is red
            while (RGB.red > RGB.blue && rollTime < 1200) { // roll until sees blue
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10);  
            }
        } else if (odometry::robot.team && !initColour) { // if team is red and inital colour is blue
            while (RGB.blue > RGB.red && rollTime < 9999999) { // roll until sees red
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10);  
            }
            while (RGB.red > RGB.blue && rollTime < 9999999) { // roll until sees blue
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10); 
            }
        } else if (!odometry::robot.team && initColour) { // if team is blue and inital colour is red
            while (RGB.red > RGB.blue && rollTime < 1200) { // roll until sees blue
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10); 
            }
            while (RGB.blue > RGB.red && rollTime < 1200) { // roll until sees red
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10);  
            }
        } else { // if team is blue and inital colour is blue
            while (RGB.blue > RGB.red && rollTime < 1200) { // roll until sees red
                RGB = colour.get_rgb(); // get colour sensor value
                rollTime += 10; // increase time to prevent infinite loop when not sensing colour
                pros::delay(10);  
            }
        }
        intake.move(0); // stop rollers
    }

    void back(bool axis) { // 0 - X, 1 = Y
        double target = (axis) ? rollerX2: rollerY1; // gets target roller position
        double error = target - ((axis) ? odometry::robot.x: odometry::robot.y); // gets difference in position

        chassis.set_drive_pid(error, backSpeed); // backs into roller
        chassis.wait_drive();
    }
}
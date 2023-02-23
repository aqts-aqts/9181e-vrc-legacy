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
    }

    void back(double maxV) { // depreciated, uses standard pid drive
        double distance = (wall.get() - rollerDist) / 25.4;
        chassis.set_drive_pid(-distance, maxV, true);
        chassis.wait_drive();
    }
}
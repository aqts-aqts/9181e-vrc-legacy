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

double error = INFINITY;
double prevError = 0;
double totalError = 0;
double derivative;

double motorPower;
double leftPower;
double rightPower;
double offset;

double kP;
double kD;
double kI;
double kC;
double p_start_i;
double c_start_i;

double small_error;
double small_exit_time;
double big_error;
double big_exit_time;
double velocity_error;
double velocity_exit_time;
double mA_timeout;

int small_t = 0;
int big_t = 0;
int velocity_t = 0;
int mA_t = 0;

bool is_over_mA = false;
bool interfered = false;

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

    void positionTrack(void* param) {
        while (true) {
            updateSensors();
            updatePosition();
            pros::delay(10);
        }
    }

    void set_approach_constants(double p, double d, double i, double c, double psi = INFINITY, double csi = INFINITY) {
        kP = p;
        kD = d;
        kI = i;
        kC = c;
        p_start_i = psi;
        c_start_i = csi;
    }

    void set_approach_exit_conditions(double small_err, double small_time, double big_err = 0, double big_time = 0, double velocity_err = 0, double velocity_time = 0, double mA_time = 0) {
        small_error = small_err;
        small_exit_time = small_time;
        big_error = big_err;
        big_exit_time = big_time;
        velocity_exit_time = velocity_time;
        mA_timeout = mA_time;
    }

    void reset_approach_exit_timers() {
        int small_t = 0;
        int big_t = 0;
        int velocity_t = 0;
        int mA_t = 0;
    }

    bool get_approach_exit_condition() {
        if (fabs(error) < small_error) {
            small_t += ez::util::DELAY_TIME;
            big_t = 0;
            if (small_t > small_exit_time) {
                reset_approach_exit_timers();
                return false;
            }
        } else {
            small_t = 0;
        }
        if (big_error != 0) {
            if (fabs(error) < big_error) {
                big_t += ez::util::DELAY_TIME;
                if (big_t > big_exit_time) {
                    reset_approach_exit_timers();
                    return false;
                }   
            } else {
                big_t = 0;
            }
        }
        if (velocity_error != 0) {
            if (fabs(derivative) < velocity_error) {
                velocity_t += ez::util::DELAY_TIME;
                if (velocity_t > velocity_exit_time) {
                    reset_approach_exit_timers();
                    interfered = true;
                    return false;
                }
            } else {
                velocity_t = 0;
            }
        }
        if (mA_timeout != 0) {
            for (auto motor: chassis.left_motors) {
                if (motor.is_over_current()) {
                    is_over_mA = true;
                    break;
                } else {
                    is_over_mA = false;
                }
            }
            if (!is_over_mA) {
                for (auto motor: chassis.right_motors) {
                    if (motor.is_over_current()) {
                        is_over_mA = true;
                        break;
                    } else {
                        is_over_mA = false;
                    }
                }
            }
            if (is_over_mA) {
                mA_t += ez::util::DELAY_TIME;
                if (mA_t > mA_timeout) {
                    reset_approach_exit_timers();
                    interfered = true;
                    return false;
                }
            } else {
                mA_t = 0;
            }
        }
        return true;
    }

    void approach_to_point(double x, double y, double maxSpeed = DRIVE_SPEED, double maxturnSpeed = TURN_SPEED) {
        double angle = chassis.imu.get_rotation() + atan2(y - robot.y, x - robot.x) * 180 / M_PI;

        chassis.set_turn_pid(angle, maxturnSpeed);
        chassis.wait_drive();

        while (get_approach_exit_condition()) {
            error = fabs(sqrt(pow(x - robot.x, 2) + pow(y - robot.y, 2)));
            totalError += error;
            derivative = error - prevError;

            if (error < p_start_i) motorPower = error * kP + derivative * kD;
            else motorPower = error * kP + derivative * kD + totalError * kI;

            offset = chassis.imu.get_rotation() - angle;

            leftPower = fmax(maxSpeed, motorPower - offset * kC);
            rightPower = fmax(maxSpeed, motorPower + offset * kC);

            if (offset < c_start_i) chassis.set_tank(leftPower, rightPower);
            else chassis.set_tank(motorPower, motorPower);
            pros::delay(10);
        }
    }
}
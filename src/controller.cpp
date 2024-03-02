#include "main.h"
using namespace global;

namespace global {
    void rotate_catapult(double velocity) {
        catapult.move_relative(-CATAPULT_ROTATION, velocity * REVERSE_CATAPULT);
    }

    void raise_lower_lift(bool lift_state, double velocity) {
        if (lift_state)
            lift.move_absolute(LIFT_HOLD, -velocity * REVERSE_LIFT);
        else
            lift.move_absolute(LIFT_STOP, velocity * REVERSE_LIFT);
    }

    void raise_lower_blocker(bool blocker_state, double velocity) {
        if (blocker_state)
            lift.move_absolute(LIFT_BLOCK, -velocity * REVERSE_LIFT);
        else
            lift.move_absolute(LIFT_STOP, velocity * REVERSE_LIFT);
    }

    void move_relative(double distance, int timeout) {
        double currentX = chassis.getPose().x;
        double currentY = chassis.getPose().y;
        double theta = chassis.getPose().theta * M_PI / 180.0;

        double x = currentX + distance * sin(theta);
        double y = currentY + distance * cos(theta);

        printf("Moving to: (%f, %f)\n", x, y);
        chassis.moveTo(x, y, timeout);
    }
    
    void turn_relative(double theta, int timeout) {
        double currentTheta = chassis.getPose().theta;
        double newTheta = (currentTheta + theta) * M_PI / 180;

        double x = chassis.getPose().x + (sin(newTheta) * 1000);
        double y = chassis.getPose().y + (cos(newTheta) * 1000);

        printf("Turning to: (%f)\n", newTheta * 180 / M_PI);
        chassis.turnTo(x, y, timeout);
    }

    void left_swing_relative(double theta, int timeout) {
        double currentTheta = chassis.getPose().theta;
        double newTheta = currentTheta + theta;
        double error = newTheta - currentTheta;
        double lastError = 0;
        double derivative;

        double kP = SWING_KP;
        double kD = SWING_KD;

        int time = 0;
        int big_error_time = 0;
        int small_error_time = 0;

        while (time < timeout) {
            currentTheta = chassis.getPose().theta;
            error = newTheta - currentTheta;
            derivative = error - lastError;

            double power = kP * error + kD * derivative;
            set_drive(power, 0);

            if (fabs(error) <= SWING_BIG_ERROR) {
                big_error_time += 10;
                if (big_error_time >= SWING_BIG_ERROR_TIMEOUT) break;
            } else {
                big_error_time = 0;
            }

            if (fabs(error) <= SWING_SMALL_ERROR) {
                small_error_time += 10;
                if (small_error_time >= SWING_SMALL_ERROR_TIMEOUT) break;
            } else {
                small_error_time = 0;
            }

            lastError = error;
            pros::delay(10);
            time += 10;
        }
        set_drive(0, 0);
        printf("Left swing finished: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    }

    void right_swing_relative(double theta, int timeout) {
        double currentTheta = chassis.getPose().theta;
        double newTheta = currentTheta + theta;
        double error = newTheta - currentTheta;
        double lastError = 0;
        double derivative;

        double kP = SWING_KP;
        double kD = SWING_KD;

        int time = 0;
        int big_error_time = 0;
        int small_error_time = 0;

        while (time < timeout) {
            currentTheta = chassis.getPose().theta;
            error = newTheta - currentTheta;
            derivative = error - lastError;

            double power = kP * error + kD * derivative;
            set_drive(0, -power);

            if (fabs(error) <= SWING_BIG_ERROR) {
                big_error_time += 10;
                if (big_error_time >= SWING_BIG_ERROR_TIMEOUT) break;
            } else {
                big_error_time = 0;
            }

            if (fabs(error) <= SWING_SMALL_ERROR) {
                small_error_time += 10;
                if (small_error_time >= SWING_SMALL_ERROR_TIMEOUT) break;
            } else {
                small_error_time = 0;
            }

            lastError = error;
            pros::delay(10);
            time += 10;
        }
        set_drive(0, 0);
        printf("Right swing finished: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    }

    void turn(double theta, int timeout) {
        double radians = theta * M_PI / 180;

        double x = chassis.getPose().x + (sin(radians) * 1000);
        double y = chassis.getPose().y + (cos(radians) * 1000);

        chassis.turnTo(x, y, timeout);
    }

    void left_swing(double theta, int timeout) {
        double currentTheta = chassis.getPose().theta;
        double error = theta - currentTheta;
        double lastError = 0;
        double derivative;

        double kP = SWING_KP;
        double kD = SWING_KD;

        int time = 0;
        int big_error_time = 0;
        int small_error_time = 0;

        while (time < timeout) {
            currentTheta = chassis.getPose().theta;
            error = theta - currentTheta;
            derivative = error - lastError;

            double power = kP * error + kD * derivative;
            set_drive(power, 0);

            if (fabs(error) <= SWING_BIG_ERROR) {
                big_error_time += 10;
                if (big_error_time >= SWING_BIG_ERROR_TIMEOUT) break;
            } else {
                big_error_time = 0;
            }

            if (fabs(error) <= SWING_SMALL_ERROR) {
                small_error_time += 10;
                if (small_error_time >= SWING_SMALL_ERROR_TIMEOUT) break;
            } else {
                small_error_time = 0;
            }

            lastError = error;
            pros::delay(10);
            time += 10;
        }
        set_drive(0, 0);
        printf("Left swing finished: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    }

    void right_swing(double theta, int timeout) {
        double currentTheta = chassis.getPose().theta;
        double error = theta - currentTheta;
        double lastError = 0;
        double derivative;

        double kP = SWING_KP;
        double kD = SWING_KD;

        int time = 0;
        int big_error_time = 0;
        int small_error_time = 0;

        while (time < timeout) {
            currentTheta = chassis.getPose().theta;
            error = theta - currentTheta;
            derivative = error - lastError;

            double power = kP * error + kD * derivative;
            set_drive(0, -power);

            if (fabs(error) <= SWING_BIG_ERROR) {
                big_error_time += 10;
                if (big_error_time >= SWING_BIG_ERROR_TIMEOUT) break;
            } else {
                big_error_time = 0;
            }

            if (fabs(error) <= SWING_SMALL_ERROR) {
                small_error_time += 10;
                if (small_error_time >= SWING_SMALL_ERROR_TIMEOUT) break;
            } else {
                small_error_time = 0;
            }

            lastError = error;
            pros::delay(10);
            time += 10;
        }
        set_drive(0, 0);
        printf("Right swing finished: (%f, %f, %f)\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    }
}
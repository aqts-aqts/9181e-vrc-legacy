#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
using namespace global;

namespace global {
    pros::Motor FW1(1, pros::E_MOTOR_GEARSET_06);
    pros::Motor FW2(11, pros::E_MOTOR_GEARSET_06);
    pros::Motor intake(10, pros::E_MOTOR_GEARSET_18);
    pros::Motor indexer(2, pros::E_MOTOR_GEARSET_36);

    pros::Distance wall(-1);
    pros::Optical colour(-1);
    pros::Vision vision(-1);

    pros::Rotation horizontalEncoder(17);
    pros::Rotation verticalEncoder(16);
    pros::ADIDigitalOut expansion(-1, 0);

    int elapsed;
    int disks;

    // flywheel pid stuff
    double error;
    double integral;
    double derivative;
    double prevError;

    void init() {
        elapsed = 0; // Time since opcontrol started

        // init flywheel pid
        error = 0;
        integral = 0;
        derivative = 0;
        prevError = 0;
        
        FW1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        FW2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        master.clear();
        horizontalEncoder.reset();
        verticalEncoder.reset();
    }

    double calculateFlywheelPower(double targetVelocity, double kP, double kI, double kD) {
        double currentVelocity = (FW1.get_actual_velocity() * reverseFW1 + FW2.get_actual_velocity() * reverseFW2) / 2;
        error = targetVelocity - currentVelocity;

        integral += error;
        derivative = error - prevError;

        double motorVelocity = kP * error + kI * integral + kD * derivative;
        prevError = error;

        pros::lcd::print(4, "Velocity: %f", currentVelocity);
        pros::lcd::print(5, "PID: %f", motorVelocity);

        return motorVelocity; 
    }

    double countDisks() {

    }
    
    void updateDisplay() {
        pros::lcd::print(0, "X: %f", odometry::robot.x);
        pros::lcd::print(1, "Y: %f", odometry::robot.y);
        pros::lcd::print(2, "Angle: %f", odometry::robot.angle);

        pros::lcd::print(6, "Feeder: %f", indexer.get_actual_velocity());
    }
}
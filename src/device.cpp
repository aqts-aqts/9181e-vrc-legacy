#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
using namespace global;

namespace global {
    pros::Motor FW1(1, pros::E_MOTOR_GEARSET_06);
    pros::Motor FW2(11, pros::E_MOTOR_GEARSET_06);
    pros::Motor intake(10, pros::E_MOTOR_GEARSET_18);
    pros::Motor indexer(2, pros::E_MOTOR_GEARSET_06);

    pros::Distance counter(-1);
    pros::Distance wall(-1);
    pros::Optical colour(-1);
    pros::Vision vision(-1);

    pros::Rotation horizontalEncoder(6);
    pros::Rotation verticalEncoder(8);
    pros::ADIDigitalOut expansion(-1, 0);

    int elapsed;
    int discs;

    // disc counting stuff
    double lastDist;

    // flywheel pid stuff
    double error;
    double integral;
    double derivative;
    double prevError;

    double targetVelocity;
    double currentVelocity;
    double lastTarget;

    double kP;
    double kI;
    double kD;
    double speedkP;
    
    double p_constant;
    bool crossed;

    int startTime;

    void init() {
        elapsed = 0; // Time since opcontrol started
        discs = 0;
        lastDist = counter.get();

        // init flywheel pid
        error = 0;
        integral = 0;
        derivative = 0;
        prevError = 0; 

        // velocity constants kP = 0.005, kI = 0.000001, kD = 0.01

        // flywheel constants
        // increase when undershooting, decrease when overshooting
        kP = 0.01; // proportional = positive when speeding up, negative when slowing down 
        kI = 0.000001; // integral = gains when under target, loses when over target
        kD = 0.01; // derivative = negative when approaching target, positive when leaving target
        speedkP = 0.005; // kP value used for speeding up

        p_constant = speedkP; // constant used for proportional control

        // flywheel status
        currentVelocity = 0;
        targetVelocity = 0;
        lastTarget = 0;

        crossed = false;
        
        FW1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        FW2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        master.clear();
        horizontalEncoder.reset();
        verticalEncoder.reset();
        horizontalEncoder.reset_position();
        verticalEncoder.reset_position();
    }

    double calculateFlywheelPID() {
        double trueVelocity = (FW1.get_actual_velocity() * reverseFW1 + FW2.get_actual_velocity() * reverseFW2) / 2;
        error = targetVelocity - trueVelocity;

        if (fabs(error) < 10 && !crossed) {
            p_constant = kP;
            crossed = true;
        } else {
            p_constant = speedkP;
        }

        integral += error;
        derivative = error - prevError;

        double motorVelocity = p_constant * error + kI * integral + kD * derivative;
        prevError = error;

        pros::lcd::print(4, "Velocity: %f", trueVelocity);
        pros::lcd::print(5, "Target: %f", targetVelocity);
        return motorVelocity; 
    }

    void flywheelPID(void* param) {
        while (true) {
            currentVelocity += calculateFlywheelPID();

            FW1.move(currentVelocity * reverseFW1);
            FW2.move(currentVelocity * reverseFW2);

            pros::delay(10);
        }
    }

    void countDiscs(void* param) {
        while (true) {
            if (counter.get() - lastDist > discWidth)
                discs += 1;
            lastDist = counter.get();

            pros::delay(10);
        }
    }
}
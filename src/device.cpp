#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/adi.hpp"
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
    pros::Optical colour(7);
    pros::Vision vision(-1);

    pros::Rotation horizontalEncoder(6);
    pros::Rotation verticalEncoder(8);
    pros::ADIDigitalOut expansionUpL(2, 0); // B
    pros::ADIDigitalOut expansionUpR(1, 0); // A
    pros::ADIDigitalOut expansionDownL(4, 0); // D
    pros::ADIDigitalOut expansionDownR(3, 0); // C

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
    bool use_pid;

    int startTime;

    void init() {
        elapsed = 0; // Time since opcontrol started
        discs = 0; // Number of discs counted
        lastDist = counter.get();

        // init flywheel pid
        error = 0;
        integral = 0;
        derivative = 0;
        prevError = 0; 

        // velocity constants kP = 0.005, kI = 0.000001, kD = 0.01

        // flywheel constants
        kP = 0.045; // proportional = positive when speeding up, negative when slowing down 
        kI = 0.000001; // integral = gains when under target, loses when over target
        kD = 0.01; // derivative = negative when approaching target, positive when leaving target
        speedkP = 0.01; // kP value used for speeding up

        p_constant = speedkP; // constant used for proportional control

        // flywheel status
        currentVelocity = 0; // current velocity
        targetVelocity = 0; // current target velocity
        lastTarget = 0; // previous target velocity
        crossed = false; // has crossed 10 error threshold
        use_pid = false;
        
        FW1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        FW2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

        master.clear();
        horizontalEncoder.reset();
        verticalEncoder.reset();
        horizontalEncoder.reset_position();
        verticalEncoder.reset_position();

        colour.set_led_pwm(25);
    }

    double calculateFlywheelPID() {
        // get average velocity of flywheel
        double trueVelocity = (FW1.get_actual_velocity() * reverseFW1 + FW2.get_actual_velocity() * reverseFW2) / 2;
        error = targetVelocity - trueVelocity;

        if (fabs(error) < 10 && !crossed) { // check if threshold has been crossed
            p_constant = kP; // switch p constant to normal
            crossed = true;
        } else {
            p_constant = speedkP; // use speed p constant to speed up faster/slower
        }

        integral += error; // add error to integral
        derivative = error - prevError; // get difference between error and previous error

        double motorVelocity = p_constant * error + kI * integral + kD * derivative; // calculate motor velocity
        prevError = error;

        pros::lcd::print(4, "Velocity: %f", trueVelocity);
        pros::lcd::print(5, "Target: %f", targetVelocity);
        pros::lcd::print(6, "P Constant: %f", p_constant);
        return motorVelocity; 
    }

    void flywheelPID(void* param) { // flywheel PID task (runs in parallel)
        while (true) {
            if (use_pid) {
                currentVelocity += calculateFlywheelPID(); // update current velocity

                FW1.move(currentVelocity * reverseFW1); // apply velocities
                FW2.move(currentVelocity * reverseFW2);
            } else {
                FW1.move(targetVelocity * 127 * reverseFW1); // apply velocities
                FW2.move(targetVelocity * 127 * reverseFW2);
            }
            pros::delay(10);
        }
    }

    void countDiscs(void* param) { // disc counting task (runs in parallel)
        while (true) {
            if (counter.get() - lastDist > discWidth) // if the counter has increased more than the width of a disc
                discs += 1;
            lastDist = counter.get(); // update last distance

            pros::delay(10);
        }
    }
}
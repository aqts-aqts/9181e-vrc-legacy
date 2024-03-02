#include "main.h"
using namespace global;

namespace global {
    /*void test() {
        chassis.setPose(0, 0, -30);
        chassis.moveTo(24, -40, 1500);
        chassis.moveTo(10, -20, 1500);
        chassis.turnTo(10, -60, 1000);
        pros::Task([]() {
            pros::delay(250);
            flaps.set_value(1);
        });
        chassis.moveTo(5, -10, 1000);
        flaps.set_value(0);
        chassis.turnTo(-40, -5, 1500);
        hold_lift(LIFT_RPM);
        chassis.moveTo(-40, -5, 1500);
    }*/

    void test() {
        // Score preloads and match load
        chassis.setPose(0, 0, -20);
        move_relative(-35, 1500);
        turn(-30, 500);
        move_relative(22, 1500);
        turn(-105, 1000);
        move_relative(-7, 500);
        flaps.set_value(1);
        // catapult.move_velocity(CATAPULT_RPM * REVERSE_CATAPULT);
        pros::delay(1000); // 30000
        flaps.set_value(0);
        catapult.move_velocity(0);

        // First push
        left_swing(-60, 1000);
        move_relative(21, 1000);
        turn(90, 1000);
        move_relative(-80, 4000);
        pros::delay(500);
        right_swing(45, 1000);
        set_drive(-127, -127);
        pros::delay(1000);
        set_drive(0, 0);
        move_relative(10, 1000);
        set_drive(-127, -127);
        pros::delay(500);
        set_drive(0, 0);
     
        // Second push
        move_relative(8, 1000);
        turn(-90, 1000);
        move_relative(-40, 1500);
        turn(0, 1000);
        move_relative(-20, 1000);
        right_swing(90, 1000);
        flaps.set_value(1);
        set_drive(-127, -127);
        pros::delay(1000);
        set_drive(0, 0);
        move_relative(10, 1000);
        set_drive(-127, -127);
        pros::delay(500);
        set_drive(0, 0);

        // Third push
        flaps.set_value(0);
        move_relative(30, 1500);
        turn(180, 1000);
        move_relative(40, 1000);
        turn(140, 1000);
        flaps.set_value(1);
        set_drive(-127, -127);
        pros::delay(1000);
        set_drive(0, 0);
        move_relative(10, 1000);
        set_drive(-127, -127);
        pros::delay(500);
        set_drive(0, 0);
        move_relative(10, 1000);
        flaps.set_value(0);
    }

    void skills() {

    }

    void awp() {

    }

    void far_side() {

    }
} 
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
using namespace global;

namespace global {  
    int elapsed;

    pros::Motor catapult(12, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor claw(11, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

    pros::ADIDigitalOut flaps(3, 0);
    pros::ADIDigitalOut climber(5, 0);

    pros::Rotation horizontalEncoder(17);
    pros::Rotation verticalEncoder(9);

    void init() {
        elapsed = 0;

        catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapult.tare_position();
        claw.tare_position();

        horizontalEncoder.reset();
        verticalEncoder.reset();
        horizontalEncoder.reset_position();
        verticalEncoder.reset_position();
    }
}
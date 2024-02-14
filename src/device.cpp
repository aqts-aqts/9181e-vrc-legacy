#include "main.h"
using namespace global;

namespace global {  
    int elapsed;

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::Motor left_front(-1, pros::E_MOTOR_GEARSET_06);
    pros::Motor left_middle(2, pros::E_MOTOR_GEARSET_06);
    pros::Motor left_back(-3, pros::E_MOTOR_GEARSET_06);
    pros::Motor right_front(5, pros::E_MOTOR_GEARSET_06);
    pros::Motor right_middle(-6, pros::E_MOTOR_GEARSET_06);
    pros::Motor right_back(7, pros::E_MOTOR_GEARSET_06);

    pros::MotorGroup left_motors({left_front, left_middle, left_back});
    pros::MotorGroup right_motors({right_front, right_middle, right_back});

    pros::Imu imu(8);

    pros::Rotation horizontal_encoder(0);
    pros::Rotation vertical_encoder(0);

    pros::Motor catapult(12, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor intake(9, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor lift(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

    pros::ADIDigitalOut flaps(3, 0);

    lemlib::TrackingWheel horizontal(&horizontal_encoder, 2.75, 2); // Tracking wheel diameter, offset from tracking center
    lemlib::TrackingWheel vertical(&vertical_encoder, 2.75, 0);

    lemlib::Drivetrain_t drivetrain {
        &left_motors, 
        &right_motors, 
        10, // Track width
        3.25, // Wheel diameter
        360 // RPM of wheels
    };

    lemlib::ChassisController_t lateral_controller {
        8, // kP
        30, // kD
        1, // smallError
        100, // smallErrorTimeout
        3, // largeError
        500, // largeErrorTimeout
        5 // slew
    };

    lemlib::ChassisController_t angular_controller {
        4, // kP
        40, // kD
        1, // smallError
        100, // smallErrorTimeout
        3, // largeError
        500, // largeErrorTimeout
        40 // slew
    };

    lemlib::OdomSensors_t sensors {
        &vertical,
        nullptr,
        &horizontal,
        nullptr,
        &imu
    };

    lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

    void init() {
        elapsed = 0;

        left_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
        right_motors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

        catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapult.tare_position();
        lift.tare_position();

        flaps.set_value(0);

        // chassis.calibrate();
        // chassis.setPose(0, 0, 0);
    }
}
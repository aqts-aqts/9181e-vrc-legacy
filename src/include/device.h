#pragma once
#include "main.h"

#define CATAPULT_RPM 100
#define INTAKE_RPM 200
#define LIFT_RPM 200

#define REVERSE_CATAPULT -1
#define REVERSE_INTAKE 1
#define REVERSE_LIFT 1

namespace global {
    extern int elapsed;

    extern pros::Controller master;

    extern pros::Motor left_front;
    extern pros::Motor left_middle;
    extern pros::Motor left_back;
    extern pros::Motor right_front;
    extern pros::Motor right_middle;
    extern pros::Motor right_back;

    extern pros::MotorGroup left_motors;
    extern pros::MotorGroup right_motors;

    extern pros::Imu imu;

    extern pros::Rotation horizontal_encoder;
    extern pros::Rotation vertical_encoder;

    extern pros::Motor catapult;
    extern pros::Motor intake;
    extern pros::Motor lift;

    extern pros::ADIDigitalOut flaps;

    extern lemlib::TrackingWheel horizontal;
    extern lemlib::TrackingWheel vertical;

    extern lemlib::Drivetrain_t drivetrain;

    extern lemlib::ChassisController_t lateral_controller;
    extern lemlib::ChassisController_t angular_controller;

    extern lemlib::OdomSensors_t sensors;

    extern lemlib::Chassis chassis;

    extern void init();
}
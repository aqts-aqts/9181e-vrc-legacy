#pragma once
#include "main.h"

#define autoToManualCooldown 1200
#define catapultCooldown 800
#define flapCooldown 500
#define clawCooldown 500
#define climberCooldown 500

#define clawLowerSpeed 1
#define clawRaiseSpeed 1
#define clawManualSpeed 1

#define clawDuration 1200

#define reverseCatapult 1
#define reverseClaw 1

#define clawRPM 100
#define catapultRPM 100

namespace global {
    extern int elapsed;

    extern pros::Motor catapult;
    extern pros::Motor claw;

    extern pros::ADIDigitalOut flaps;
    extern pros::ADIDigitalOut climber;

    extern pros::Rotation horizontalEncoder;
    extern pros::Rotation verticalEncoder;

    extern void init();
}
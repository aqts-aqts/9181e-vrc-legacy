#pragma once
#include "main.h"

#define reverseFW1 -1
#define reverseFW2 1
#define reverseIntake -1
#define reverseIndexer -1

#define aimTurnSpeed 117 // easier turning as more power can be allocated to turn rather than just lowering
#define flywheelGearRatio 7.5 // ratio between motor power and angular velocity of flywheel

#define indexerFeedSpeed 100
#define intakeFeedSpeed 127
#define discWidth 25


namespace global {
    extern pros::Motor FW1;
    extern pros::Motor FW2;
    extern pros::Motor intake;
    extern pros::Motor indexer;

    extern pros::Distance counter;
    extern pros::Distance wall;
    extern pros::Optical colour;
    extern pros::Vision vision;
    extern pros::ADIDigitalOut expansion;
    
    extern pros::Rotation horizontalEncoder;
    extern pros::Rotation verticalEncoder;

    extern void init();
    extern void updateDisplay();
    extern void countDiscs();
    extern double calculateFlywheelPower();
    extern void flywheelPID(void* param);

    extern int elapsed;
    extern int discs;

    extern double targetVelocity;
    extern double currentVelocity;
    extern double lastTarget;
}
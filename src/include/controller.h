#pragma once
#include "main.h"

#define CATAPULT_ROTATION 360
#define LIFT_STOP 0
#define LIFT_HOLD 90
#define LIFT_BAR 220
#define LIFT_BLOCK 400

#define AUTO_TO_MANUAL_COOLDOWN 1200
#define CATAPULT_COOLDOWN 800
#define LIFT_COOLDOWN 500
#define FLAP_COOLDOWN 500

#define SWING_KP 7
#define SWING_KD 45
#define SWING_BIG_ERROR 0.7
#define SWING_BIG_ERROR_TIMEOUT 20
#define SWING_SMALL_ERROR 0.3
#define SWING_SMALL_ERROR_TIMEOUT 10

namespace global {
    extern void rotate_catapult(double velocity);
    extern void raise_lower_lift(bool lift_state, double velocity);
    extern void raise_lower_blocker(bool blocker_state, double velocity);

    extern void move_relative(double distance, int timeout);
    extern void turn_relative(double theta, int timeout);
    extern void left_swing_relative(double theta, int timeout);
    extern void right_swing_relative(double theta, int timeout);

    extern void turn(double theta, int timeout);
    extern void left_swing(double theta, int timeout);
    extern void right_swing(double theta, int timeout);
}
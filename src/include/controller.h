#pragma once
#include "main.h"

#define CATAPULT_ROTATION 360
#define LIFT_STOP 0
#define LIFT_HOLD 90
#define LIFT_BLOCK 400

#define AUTO_TO_MANUAL_COOLDOWN 1200
#define CATAPULT_COOLDOWN 800
#define LIFT_COOLDOWN 500
#define FLAP_COOLDOWN 500

namespace global {
    extern void rotate_catapult(double velocity);
    extern void raise_lower_lift(bool lift_state, double velocity);
    extern void raise_lower_blocker(bool blocker_state, double velocity);
}
#pragma once
#include "main.h"

#define catapultReset -360
#define clawStop 0
#define clawReset -420

namespace global {
    void activate_catapult(double velocity);
    void deactivate_catapult();
    void activate_claw(double velocity);
    void deactivate_claw(double velocity);
}
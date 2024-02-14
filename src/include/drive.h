#pragma once
#include "main.h"

#define DEADZONE 5
#define ACTIVE_BRAKE_KP 0.003

namespace global {
    extern void drive();
    extern void set_drive(double left, double right);
    extern double apply_threshold(double input);
}
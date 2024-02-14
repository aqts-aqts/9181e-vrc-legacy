#pragma once
#include "main.h"

#define LEFT_CURVE_SCALE 0
#define RIGHT_CURVE_SCALE 0

#define WHEEL_DIAMETER 3.25
#define DRIVE_CARTRIDGE_RPM 600
#define DRIVE_RATIO 1.6667 // 60:36

namespace global {
    extern double curve_left(double input);
    extern double curve_right(double input);
    
    extern double get_drive_tick_per_inch();
    extern double get_left_drive_sensor();
    extern double get_right_drive_sensor();
}
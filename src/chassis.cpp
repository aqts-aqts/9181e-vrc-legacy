#include "main.h"
using namespace global;

namespace global {
    double curve_left(double input) {
        if (LEFT_CURVE_SCALE == 0)
            return input;
        return (powf(2.718, -(LEFT_CURVE_SCALE / 10.0)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(LEFT_CURVE_SCALE / 10.0)))) * input;
    }

    double curve_right(double input) {
        if (RIGHT_CURVE_SCALE == 0)
            return input;
        return (powf(2.718, -(RIGHT_CURVE_SCALE / 10.0)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(RIGHT_CURVE_SCALE / 10.0)))) * input;
    }

    double get_drive_tick_per_inch() {
        double circumference = WHEEL_DIAMETER * M_PI;
        double tick_per_revolution = DRIVE_CARTRIDGE_RPM * DRIVE_RATIO;

        return tick_per_revolution / circumference;
    }

    double get_left_drive_raw() {
        std::vector<double> positions = left_motors.get_positions();
        return (positions[0] + positions[1] + positions[2]) / 3;
    }

    double get_right_drive_raw() {
        std::vector<double> positions = right_motors.get_positions();
        return (positions[0] + positions[1] + positions[2]) / 3;
    }

    double get_left_drive_sensor() {
        return get_left_drive_raw() / get_drive_tick_per_inch();
    }

    double get_right_drive_sensor() {
        return get_right_drive_raw() / get_drive_tick_per_inch();
    }
}
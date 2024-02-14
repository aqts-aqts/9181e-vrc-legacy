#include "main.h"
using namespace global;

namespace global {
    void drive() {
        double forward_stick = curve_left(apply_threshold(master.get_analog(ANALOG_LEFT_Y)));
        double turn_stick = curve_right(apply_threshold(master.get_analog(ANALOG_LEFT_X)));

        if (fabs(forward_stick) > 0 || fabs(turn_stick) > 0)
            set_drive(forward_stick + turn_stick, forward_stick - turn_stick);
        else
            set_drive((0 - get_left_drive_sensor()) * ACTIVE_BRAKE_KP, (0 - get_right_drive_sensor()) * ACTIVE_BRAKE_KP);
    }

    void set_drive(double left, double right) {
        left_motors.move_voltage(left * (12000.0 / 127.0));
        right_motors.move_voltage(right * (12000.0 / 127.0));
    }

    double apply_threshold(double input) {
        if (fabs(input) < DEADZONE)
            return 0;
        return input;
    }
}
#include "main.h"
using namespace global;

namespace global {
    void rotate_catapult(double velocity) {
        catapult.move_relative(-CATAPULT_ROTATION, velocity * REVERSE_CATAPULT);
    }

    void raise_lower_lift(bool lift_state, double velocity) {
        if (lift_state)
            lift.move_absolute(LIFT_HOLD, -velocity * REVERSE_LIFT);
        else
            lift.move_absolute(LIFT_STOP, velocity * REVERSE_LIFT);
    }

    void raise_lower_blocker(bool blocker_state, double velocity) {
        if (blocker_state)
            lift.move_absolute(LIFT_BLOCK, -velocity * REVERSE_LIFT);
        else
            lift.move_absolute(LIFT_STOP, velocity * REVERSE_LIFT);
    }
}
#include "main.h"
using namespace global;

namespace global {
    void activate_catapult(double velocity) {
        catapult.move_relative(-catapultReset, velocity * reverseCatapult);
    }

    void activate_claw(double velocity) {
        claw.move_absolute(clawStop, velocity * reverseClaw);
    }

    void deactivate_claw(double velocity) {
        claw.move_absolute(clawReset, -velocity * reverseClaw);
    }
}
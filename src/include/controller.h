#pragma once
#include "main.h"

#define intakeVolts 127
#define indexerVolts -127
#define rollerVolts -127

#define rollerDist 70

#define blueGoalX 0 // x coordinate of blue goal
#define blueGoalY 0 // y coordinate of blue goal
#define redGoalX 0 // x coordinate of red goal
#define redGoalY 0 // y coordinate of red goal

#define aimSpeed 90

namespace global {
    void aim();
    void back(double maxV);
}
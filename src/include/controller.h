#pragma once
#include "main.h"

#define rollerDist 70

#define blueGoalX 0 // x coordinate of blue goal
#define blueGoalY 0 // y coordinate of blue goal
#define redGoalX 0 // x coordinate of red goal
#define redGoalY 0 // y coordinate of red goal

#define rollerY1 0 // y coordinate of first roller
#define rollerX2 0 // x coordinate of second roller

#define aimSpeed 90
#define backSpeed 80

namespace global {
    void aim();
    void roll(double rollerSpeed);
    void back(bool axis);
}
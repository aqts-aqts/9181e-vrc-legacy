#pragma once

#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

void default_constants();
void exit_condition_defaults();
void modified_exit_condition();

// main routes
void left_side();
void right_side();
void solo_awp();
void programmingSkills();

// extensions
void closePreload();
void shootFirst();
void threeDiscs();
void test();
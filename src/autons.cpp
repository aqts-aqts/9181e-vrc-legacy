#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/motors.h"
using namespace global;

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 90;
const int SWING_SPEED = 90;

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void programmingSkills() {
  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-23, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 66, SWING_SPEED);
  chassis.wait_drive();

  catapult.move(127);
  pros::delay(30000);
  catapult.move(0);

  chassis.set_swing_pid(LEFT_SWING, 140, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 88, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(70, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-25, 70, true);
  chassis.wait_drive();

  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(23, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();

  flaps.set_value(1);

  chassis.set_drive_pid(-36, 127, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 270, 127);
  chassis.wait_drive();

  /*chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-25, 127, true);
  chassis.wait_drive();*/

  flaps.set_value(0);

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  flaps.set_value(1);

  chassis.set_turn_pid(630, TURN_SPEED);
  chassis.wait_drive();

  flaps.set_value(0);

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(540, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-27, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(317, TURN_SPEED);
  chassis.wait_drive();

  flaps.set_value(1);
  
  chassis.set_drive_pid(-35, 127, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 270, 127);
  chassis.wait_drive();

  flaps.set_value(0);

  chassis.set_drive_pid(8, DRIVE_SPEED, true);
  chassis.wait_drive();

  /*chassis.set_drive_pid(-25, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();*/

  // second push starting here
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(50, DRIVE_SPEED, true);
  chassis.wait_drive();

  flaps.set_value(1);
  pros::delay(500);

  chassis.set_turn_pid(180, TURN_SPEED);
  chassis.wait_drive();

  flaps.set_value(0);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-60, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-8, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();

  /*chassis.set_drive_pid(-12, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();*/

  /*chassis.set_drive_pid(-20, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-20, 127, true);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();*/

  /*chassis.set_turn_pid(-40, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-35, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();*/
}

void driverSkills() { // NOTE: EVERYTHING IS REVERSED
  chassis.set_drive_pid(-20, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
  
  chassis.set_drive_pid(23, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -59, SWING_SPEED);
  chassis.wait_drive();

  catapult.move(127);
  pros::delay(35000);
  catapult.move(0);

  chassis.set_swing_pid(LEFT_SWING, -140, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, -88, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-70, DRIVE_SPEED, true);
  chassis.wait_drive();
}

void AWP() {
  chassis.set_drive_pid(32, 127, true);
  chassis.wait_drive();

  chassis.set_turn_pid(5, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-27, DRIVE_SPEED, true);
  chassis.wait_drive();

  flaps.set_value(1);
  pros::delay(500);

  chassis.set_turn_pid(-90, 127);
  chassis.wait_drive();

  flaps.set_value(0);
  pros::delay(500);

  chassis.set_swing_pid(LEFT_SWING, -150, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, -60, SWING_SPEED);
  chassis.wait_drive();

  flaps.set_value(1);

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
}

void AS() {
  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(-15, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-94, TURN_SPEED);
  chassis.wait_drive();

  deactivate_claw(100);
  pros::delay(1000);

  chassis.set_drive_pid(44, DRIVE_SPEED, true);
  chassis.wait_drive();

  hold_claw(100);
  pros::delay(1000);

  chassis.set_turn_pid(-30, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(4, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, 60, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();

  deactivate_claw(100);
  pros::delay(1000);

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-8);
  activate_claw(100);
  chassis.wait_drive();

  chassis.set_turn_pid(240, TURN_SPEED);
  chassis.wait_drive();

  flaps.set_value(1);
  pros::delay(500);

  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(20, DRIVE_SPEED, true);
  chassis.wait_drive();
}

void old_AWP() {
  chassis.set_drive_pid(40, 127, true);
  chassis.wait_drive();

  chassis.set_turn_pid(5, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-27, DRIVE_SPEED, true);
  chassis.wait_drive();

  flaps.set_value(1);
  pros::delay(500);

  chassis.set_turn_pid(-90, 127);
  chassis.wait_drive();

  flaps.set_value(0);
  pros::delay(500);

  chassis.set_swing_pid(RIGHT_SWING, 20, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(LEFT_SWING, -60, SWING_SPEED);
  chassis.wait_drive();

  flaps.set_value(1);

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
  chassis.wait_drive();
}

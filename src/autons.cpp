#include "autons.hpp"
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

void left_side() { // motor set 2
  use_pid = true;

  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  roll(60);

  chassis.set_drive_pid(2, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 54, SWING_SPEED); // turn to move to middle
  chassis.wait_drive();

  targetVelocity = 465;

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(25, 40, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-37, TURN_SPEED); // face goal
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();

  for (int i = 0; i < 3; i++) {
    indexer.move(-indexerFeedSpeed * reverseIndexer);
    pros::delay(320);
    indexer.move(indexerFeedSpeed * reverseIndexer);
    pros::delay(320);
  }

  targetVelocity = 480;

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_swing_pid(ez::RIGHT_SWING, 41, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(60, 80, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -63, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();

  for (int i = 0; i < 3; i++) {
    indexer.move(-indexerFeedSpeed * reverseIndexer);
    pros::delay(370);
    indexer.move(indexerFeedSpeed * reverseIndexer);
    pros::delay(370);
  }

  targetVelocity = 0;

  intake.move(0);
}

void right_side() {
}

void solo_awp() {
  use_pid = true;

  intake.move(-80 * reverseIntake); // roll roller

  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  intake.move(0);

  chassis.set_drive_pid(2, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 52, SWING_SPEED); // turn to move to middle
  chassis.wait_drive();

  targetVelocity = 560;

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(28, 40, true);
  chassis.wait_drive();

  pros::delay(100); // delay

  chassis.set_turn_pid(-35, TURN_SPEED); // face goal
  chassis.wait_drive();

  chassis.set_drive_pid(9, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  targetVelocity = 0;

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 44, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(78, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(-80 * reverseIntake);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-7, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_drive_pid(2, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(0);
}

void programmingSkills() { // n = 0.001 (use motor set 1)
  use_pid = false;

  double n = 0.001;

  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("\nCheckpoint 1 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 1 Y: %f\n", odometry::robot.y);

  roll(60);

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake); // start intake
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 2 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 2 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  targetVelocity = 0.763 + n; // 77

  intake.move(0);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("\nCheckpoint 3 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 3 Y: %f\n", odometry::robot.y);
  
  roll(60);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(36, DRIVE_SPEED, true); // 40
  chassis.wait_drive();

  printf("\nCheckpoint 4 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 4 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(-5, TURN_SPEED); // 0
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.768 + n; // 77

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
  
  chassis.set_drive_pid(-27, DRIVE_SPEED, true); // -31
  chassis.wait_drive();

  printf("\nCheckpoint 5 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 5 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(47, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(53.5, 70, true);
  chassis.wait_drive();

  printf("\nCheckpoint 6 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 6 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 7 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 7 Y: %f\n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  targetVelocity = 0.763 + n; // 77

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 8 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 8 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, 80, true);
  chassis.wait_drive();

  printf("\nCheckpoint 9 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 9 Y: %f\n", odometry::robot.y);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(24, 40, true); // 24
  chassis.wait_drive();

  printf("\nCheckpoint 10 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 10 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(34, DRIVE_SPEED, true); // 34
  chassis.wait_drive();

  printf("\nCheckpoint 11 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 11 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(-94, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_turn_pid(92.5, TURN_SPEED); // 92.5
  chassis.wait_drive();

  chassis.set_drive_pid(56, DRIVE_SPEED, true); // 56
  chassis.wait_drive();

  printf("\nCheckpoint 11 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 11 Y: %f\n", odometry::robot.y);

  chassis.set_swing_pid(ez::RIGHT_SWING, 180, SWING_SPEED);
  chassis.wait_drive();

  pros::delay(200); // delay a bit to let disc intake
  intake.move(0);

  chassis.set_drive_pid(-5.5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 12 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 12 Y: %f\n", odometry::robot.y);

  roll(60);

  chassis.set_swing_pid(ez::RIGHT_SWING, 142, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 13 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 13 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);

  chassis.set_drive_pid(-7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 14 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 14 Y: %f\n", odometry::robot.y);

  roll(60);

  chassis.set_drive_pid(20, DRIVE_SPEED, true);

  chassis.wait_until(5);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.wait_drive();

  printf("\nCheckpoint 15 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 15 Y: %f\n", odometry::robot.y);

  chassis.set_drive_pid(26, 44, true);
  chassis.wait_drive();

  printf("\nCheckpoint 16 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 16 Y: %f\n", odometry::robot.y);

  targetVelocity = 0.758 + n; // 77

  chassis.set_drive_pid(-41, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 17 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 17 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(179, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(46, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 18 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 18 Y: %f\n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIntake);

  targetVelocity = 0.76 + n; // 77

  chassis.set_drive_pid(-37, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 19 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 19 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(60, 70, true);
  chassis.wait_drive();

  printf("\nCheckpoint 20 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 20 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 21 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 21 Y: %f\n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.76 + n;

  chassis.set_drive_pid(-2, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 22 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 22 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 23 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 23 Y: %f\n", odometry::robot.y);

  chassis.set_drive_pid(26, 60, true);
  chassis.wait_drive();

  printf("\nCheckpoint 24 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 24 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(89, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 25 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 25 Y: %f\n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.77 + n;

  chassis.set_turn_pid(-71, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  printf("\nCheckpoint 26 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 26 Y: %f\n", odometry::robot.y);

  chassis.set_drive_pid(26, 80, true);
  chassis.wait_drive();

  printf("\nCheckpoint 27 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 27 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("\nCheckpoint 28 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 28 Y: %f\n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(0);
  indexer.move(0);

  chassis.set_drive_pid(-63, DRIVE_SPEED, true); // -60
  chassis.wait_drive();

  printf("\nCheckpoint 29 X: %f\n", odometry::robot.x);
  printf("\nCheckpoint 29 Y: %f\n", odometry::robot.y);

  chassis.set_turn_pid(43, TURN_SPEED); // 58
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true); // 3
  chassis.wait_drive();

  expansionUpL.set_value(1);
  expansionDownL.set_value(1);
  expansionUpR.set_value(1);
  expansionDownR.set_value(1);
}

void test() {
  roll(60);
}
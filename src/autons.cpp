#include "autons.hpp"
#include "main.h"
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

void left_side() {
  pros::Task flywheelControl(flywheelPID, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Control Task"); // Start flywheel control task

  chassis.set_drive_pid(-4, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  roll(80);

  chassis.set_drive_pid(2, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 54, SWING_SPEED); // turn to move to middle
  chassis.wait_drive();

  targetVelocity = 545;

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(25, 40, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-38, TURN_SPEED); // face goal
  chassis.wait_drive();

  chassis.set_drive_pid(10, DRIVE_SPEED, true);
  chassis.wait_drive();

  for (int i = 0; i < 3; i++) {
    indexer.move(-indexerFeedSpeed * reverseIndexer);
    pros::delay(400);
    indexer.move(indexerFeedSpeed * reverseIndexer);
    pros::delay(800);
  }

  targetVelocity = 600;

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_swing_pid(ez::RIGHT_SWING, 42, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(60, 80, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, -61, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(16, DRIVE_SPEED, true);
  chassis.wait_drive();

  for (int i = 0; i < 3; i++) {
    indexer.move(-indexerFeedSpeed * reverseIndexer);
    pros::delay(400);
    indexer.move(indexerFeedSpeed * reverseIndexer);
    pros::delay(800);
  }

  targetVelocity = 0;

  intake.move(0);
}

void solo_awp() {
  pros::Task flywheelControl(flywheelPID, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Control Task"); // Start flywheel control task

  intake.move(-80 * reverseIntake); // roll roller

  chassis.set_drive_pid(-4, DRIVE_SPEED, true); // back into roller
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

void programmingSkills1() {
  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("Checkpoint 1 X: %f \n", odometry::robot.x);
  printf("Checkpoint 1 Y: %f \n", odometry::robot.y);

  roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake); // start intake
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 2 X: %f \n", odometry::robot.x);
  printf("Checkpoint 2 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  targetVelocity = 420;

  intake.move(0);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("Checkpoint 3 X: %f \n", odometry::robot.x);
  printf("Checkpoint 3 Y: %f \n", odometry::robot.y);
  
  roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 4 X: %f \n", odometry::robot.x);
  printf("Checkpoint 4 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-5, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer); // shoot
  pros::delay(1200);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  targetVelocity = 420;

  chassis.set_drive_pid(-31, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 5 X: %f \n", odometry::robot.x);
  printf("Checkpoint 5 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(47, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(54, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 6 X: %f \n", odometry::robot.x);
  printf("Checkpoint 6 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 7 X: %f \n", odometry::robot.x);
  printf("Checkpoint 7 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  targetVelocity = 400;

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 8 X: %f \n", odometry::robot.x);
  printf("Checkpoint 8 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(44, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 9 X: %f \n", odometry::robot.x);
  printf("Checkpoint 9 Y: %f \n", odometry::robot.y);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(24, 44, true);
  chassis.wait_drive();

  printf("Checkpoint 10 X: %f \n", odometry::robot.x);
  printf("Checkpoint 10 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(34, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 11 X: %f \n", odometry::robot.x);
  printf("Checkpoint 11 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-95, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_turn_pid(92.5, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(56, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 11 X: %f \n", odometry::robot.x);
  printf("Checkpoint 11 Y: %f \n", odometry::robot.y);

  chassis.set_swing_pid(ez::RIGHT_SWING, 180, SWING_SPEED);
  chassis.wait_drive();

  intake.move(0);

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 12 X: %f \n", odometry::robot.x);
  printf("Checkpoint 12 Y: %f \n", odometry::robot.y);

  // roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, 142, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 13 X: %f \n", odometry::robot.x);
  printf("Checkpoint 13 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 14 X: %f \n", odometry::robot.x);
  printf("Checkpoint 14 Y: %f \n", odometry::robot.y);

  // roll(80);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);

  chassis.wait_until(5);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.wait_drive();

  printf("Checkpoint 15 X: %f \n", odometry::robot.x);
  printf("Checkpoint 15 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(22, 44, true);
  chassis.wait_drive();

  printf("Checkpoint 16 X: %f \n", odometry::robot.x);
  printf("Checkpoint 16 Y: %f \n", odometry::robot.y);

  targetVelocity = 405;

  chassis.set_drive_pid(-39, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 17 X: %f \n", odometry::robot.x);
  printf("Checkpoint 17 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(179, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(49, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 18 X: %f \n", odometry::robot.x);
  printf("Checkpoint 18 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIntake);

  targetVelocity = 405;

  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 19 X: %f \n", odometry::robot.x);
  printf("Checkpoint 19 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(60, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 20 X: %f \n", odometry::robot.x);
  printf("Checkpoint 20 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 21 X: %f \n", odometry::robot.x);
  printf("Checkpoint 21 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 400;

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 22 X: %f \n", odometry::robot.x);
  printf("Checkpoint 22 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 23 X: %f \n", odometry::robot.x);
  printf("Checkpoint 23 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(22, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 24 X: %f \n", odometry::robot.x);
  printf("Checkpoint 24 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 25 X: %f \n", odometry::robot.x);
  printf("Checkpoint 25 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 420;

  chassis.set_turn_pid(-72, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(42, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  printf("Checkpoint 26 X: %f \n", odometry::robot.x);
  printf("Checkpoint 26 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(24, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 27 X: %f \n", odometry::robot.x);
  printf("Checkpoint 27 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 28 X: %f \n", odometry::robot.x);
  printf("Checkpoint 28 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(0);
  indexer.move(0);

  chassis.set_drive_pid(-56, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 29 X: %f \n", odometry::robot.x);
  printf("Checkpoint 29 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(54, TURN_SPEED);
  chassis.wait_drive();

  expansionUp.set_value(1);
  expansionDown.set_value(1);
}

void programmingSkills() {
  pros::Task flywheelControl(flywheelTask, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Control Task"); // Start flywheel control task

  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("Checkpoint 1 X: %f \n", odometry::robot.x);
  printf("Checkpoint 1 Y: %f \n", odometry::robot.y);

  roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake); // start intake
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 2 X: %f \n", odometry::robot.x);
  printf("Checkpoint 2 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  targetVelocity = 0.775;

  intake.move(0);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  printf("Checkpoint 3 X: %f \n", odometry::robot.x);
  printf("Checkpoint 3 Y: %f \n", odometry::robot.y);
  
  roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 4 X: %f \n", odometry::robot.x);
  printf("Checkpoint 4 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-5, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer); // shoot
  pros::delay(1200);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.775;

  chassis.set_drive_pid(-31, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 5 X: %f \n", odometry::robot.x);
  printf("Checkpoint 5 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(47, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(54, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 6 X: %f \n", odometry::robot.x);
  printf("Checkpoint 6 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 7 X: %f \n", odometry::robot.x);
  printf("Checkpoint 7 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  targetVelocity = 0.775;

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 8 X: %f \n", odometry::robot.x);
  printf("Checkpoint 8 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(44, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 9 X: %f \n", odometry::robot.x);
  printf("Checkpoint 9 Y: %f \n", odometry::robot.y);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_drive_pid(24, 44, true);
  chassis.wait_drive();

  printf("Checkpoint 10 X: %f \n", odometry::robot.x);
  printf("Checkpoint 10 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(34, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 11 X: %f \n", odometry::robot.x);
  printf("Checkpoint 11 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(-95, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.set_turn_pid(92.5, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(56, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 11 X: %f \n", odometry::robot.x);
  printf("Checkpoint 11 Y: %f \n", odometry::robot.y);

  chassis.set_swing_pid(ez::RIGHT_SWING, 180, SWING_SPEED);
  chassis.wait_drive();

  intake.move(0);

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 12 X: %f \n", odometry::robot.x);
  printf("Checkpoint 12 Y: %f \n", odometry::robot.y);

  roll(80);

  chassis.set_swing_pid(ez::RIGHT_SWING, 142, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(21, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 13 X: %f \n", odometry::robot.x);
  printf("Checkpoint 13 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(270, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-indexerFeedSpeed * reverseIndexer);

  chassis.set_drive_pid(-6, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 14 X: %f \n", odometry::robot.x);
  printf("Checkpoint 14 Y: %f \n", odometry::robot.y);

  roll(80);

  chassis.set_drive_pid(21, DRIVE_SPEED, true);

  chassis.wait_until(5);

  intake.move(127 * reverseIntake);
  indexer.move(127 * reverseIndexer);

  chassis.wait_drive();

  printf("Checkpoint 15 X: %f \n", odometry::robot.x);
  printf("Checkpoint 15 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(22, 44, true);
  chassis.wait_drive();

  printf("Checkpoint 16 X: %f \n", odometry::robot.x);
  printf("Checkpoint 16 Y: %f \n", odometry::robot.y);

  targetVelocity = 0.775;

  chassis.set_drive_pid(-39, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 17 X: %f \n", odometry::robot.x);
  printf("Checkpoint 17 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(179, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(49, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 18 X: %f \n", odometry::robot.x);
  printf("Checkpoint 18 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIntake);

  targetVelocity = 0.775;

  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 19 X: %f \n", odometry::robot.x);
  printf("Checkpoint 19 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(60, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 20 X: %f \n", odometry::robot.x);
  printf("Checkpoint 20 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 21 X: %f \n", odometry::robot.x);
  printf("Checkpoint 21 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.775;

  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 22 X: %f \n", odometry::robot.x);
  printf("Checkpoint 22 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 23 X: %f \n", odometry::robot.x);
  printf("Checkpoint 23 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(22, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 24 X: %f \n", odometry::robot.x);
  printf("Checkpoint 24 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(32, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 25 X: %f \n", odometry::robot.x);
  printf("Checkpoint 25 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  indexer.move(127 * reverseIndexer);

  targetVelocity = 0.775;

  chassis.set_turn_pid(-72, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(42, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  printf("Checkpoint 26 X: %f \n", odometry::robot.x);
  printf("Checkpoint 26 Y: %f \n", odometry::robot.y);

  chassis.set_drive_pid(24, 80, true);
  chassis.wait_drive();

  printf("Checkpoint 27 X: %f \n", odometry::robot.x);
  printf("Checkpoint 27 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 28 X: %f \n", odometry::robot.x);
  printf("Checkpoint 28 Y: %f \n", odometry::robot.y);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(1200);

  intake.move(0);
  indexer.move(0);

  chassis.set_drive_pid(-56, DRIVE_SPEED, true);
  chassis.wait_drive();

  printf("Checkpoint 29 X: %f \n", odometry::robot.x);
  printf("Checkpoint 29 Y: %f \n", odometry::robot.y);

  chassis.set_turn_pid(54, TURN_SPEED);
  chassis.wait_drive();

  expansionUp.set_value(1);
  expansionDown.set_value(1);
}

void test() {
}
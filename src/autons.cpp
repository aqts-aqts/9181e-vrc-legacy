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
  indexer.move(60 * reverseIndexer); // roll roller

  chassis.set_drive_pid(-4, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();
  indexer.move(0);

  chassis.set_drive_pid(2, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 51, SWING_SPEED); // turn to move to middle
  chassis.wait_drive();

  FW1.move(0.92 * 127 * reverseFW1); // start flywheel
  FW2.move(0.92 * 127 * reverseFW2);

  intake.move(127 * reverseIntake);
  indexer.move(-127 * reverseIndexer);

  chassis.set_drive_pid(47, 88, true);
  chassis.wait_drive();

  pros::delay(100); // delay

  chassis.set_turn_pid(-38, TURN_SPEED); // face goal
  chassis.wait_drive();

  chassis.set_drive_pid(9, DRIVE_SPEED, true);
  chassis.wait_until(1);
  indexer.move(0);
  chassis.wait_until(2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(100);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);

  FW1.move(0.97 * 127 * reverseFW1); // start flywheel
  FW2.move(0.97 * 127 * reverseFW2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(200);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);
  
  FW1.move(0.97 * 127 * reverseFW1); // start flywheel
  FW2.move(0.97 * 127 * reverseFW2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(400);
  indexer.move(0);
  pros::delay(200);

  FW1.move(0.97 * 127 * reverseFW1); // start flywheel
  FW2.move(0.97 * 127 * reverseFW2);

  intake.move(127 * reverseIntake);
  indexer.move(-127 * reverseIndexer);

  chassis.set_swing_pid(ez::RIGHT_SWING, 44, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(54, 99, true);
  chassis.wait_drive();

  indexer.move(0);

  chassis.set_swing_pid(ez::LEFT_SWING, -60, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(16, DRIVE_SPEED, true);
  chassis.wait_until(2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(100);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(200);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);
  
  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(400);
  indexer.move(0);
  pros::delay(200);

  FW1.move(0);
  FW2.move(0);
  intake.move(0);
}

void solo_awp() {
  indexer.move(60 * reverseIndexer); // roll roller

  chassis.set_drive_pid(-4, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();
  indexer.move(0);

  chassis.set_drive_pid(2, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 51, SWING_SPEED); // turn to move to middle
  chassis.wait_drive();

  FW1.move(0.92 * 127 * reverseFW1); // start flywheel
  FW2.move(0.92 * 127 * reverseFW2);

  intake.move(127 * reverseIntake);
  indexer.move(-127 * reverseIndexer);

  chassis.set_drive_pid(47, 88, true);
  chassis.wait_drive();

  pros::delay(100); // delay

  chassis.set_turn_pid(-38, TURN_SPEED); // face goal
  chassis.wait_drive();

  chassis.set_drive_pid(9, DRIVE_SPEED, true);
  chassis.wait_until(1);
  indexer.move(0);
  chassis.wait_until(2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(100);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);

  FW1.move(0.97 * 127 * reverseFW1); // start flywheel
  FW2.move(0.97 * 127 * reverseFW2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(200);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);
  
  FW1.move(0.97 * 127 * reverseFW1); // start flywheel
  FW2.move(0.97 * 127 * reverseFW2);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(400);
  indexer.move(0);
  pros::delay(200);

  intake.move(127 * reverseIntake);
  indexer.move(-127 * reverseIndexer);

  FW1.move(0); // stop flywheel
  FW2.move(0);

  chassis.set_drive_pid(-12, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::LEFT_SWING, 44, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(78, DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move(0);
  indexer.move(60 * reverseIndexer);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-7, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_drive_pid(1, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(0);
}

void programmingSkills() {
  indexer.move(-80 * reverseIndexer); // roll roller

  chassis.set_drive_pid(-3, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, SWING_SPEED);
  chassis.wait_drive();

  intake.move(127 * reverseIntake); // start intake

  chassis.set_drive_pid(28, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  FW1.move(0.845 * 127 * reverseFW1); // start flywheel
  FW2.move(0.845 * 127 * reverseFW2);

  chassis.set_drive_pid(-4, DRIVE_SPEED, true); // back into roller
  chassis.wait_drive();

  indexer.move(0);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(38, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer); // shoot
  pros::delay(1100);
  
  indexer.move(-127 * reverseIntake);

  FW1.move(0.815 * 127 * reverseFW1); // start flywheel
  FW2.move(0.815 * 127 * reverseFW2);

  chassis.set_drive_pid(-38, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  pros::delay(200);

  chassis.set_drive_pid(59, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer);
  pros::delay(1100);

  indexer.move(-127 * reverseIndexer);

  FW1.move(0.854 * 127 * reverseFW1); // start flywheel
  FW2.move(0.854 * 127 * reverseFW2);

  chassis.set_drive_pid(-7, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(22, 80, true);
  chassis.wait_drive();

  indexer.move(0);

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(38, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer);
  pros::delay(1100);

  intake.move(0);
  indexer.move(-60 * reverseIndexer);

  chassis.set_drive_pid(-45.5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-180, SWING_SPEED);
  chassis.wait_drive();

  // chassis.set_drive_pid(-7, DRIVE_SPEED, true);
  // chassis.wait_drive();
  chassis.set_drive_pid(-3, DRIVE_SPEED, true);
  chassis.wait_drive();
  back(DRIVE_SPEED);

  chassis.set_swing_pid(ez::RIGHT_SWING, -218, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(27, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(-60 * reverseIndexer);

  // chassis.set_drive_pid(-5, DRIVE_SPEED, true); // back into roller
  // chassis.wait_drive();
  chassis.set_drive_pid(-3, DRIVE_SPEED, true);
  chassis.wait_drive();
  back(DRIVE_SPEED);

  intake.move(127 * reverseIntake);

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  pros::delay(500);

  indexer.move(-90 * reverseIndexer);
  chassis.wait_drive();
  
  chassis.set_drive_pid(20, 80, true);
  chassis.wait_drive();

  FW1.move(0.815 * 127 * reverseFW1); // start flywheel
  FW2.move(0.815 * 127 * reverseFW2);

  chassis.set_drive_pid(-34, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-180, TURN_SPEED);
  chassis.wait_drive();
  
  chassis.set_drive_pid(42, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer); // shoot
  pros::delay(1100);

  indexer.move(-127 * reverseIntake);

  FW1.move(0.791 * 127 * reverseFW1); // start flywheel
  FW2.move(0.791 * 127 * reverseFW2);

  chassis.set_drive_pid(-38, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(57.5, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-223, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer);
  pros::delay(1100);

  indexer.move(-127 * reverseIndexer);

  FW1.move(0.83 * 127 * reverseFW1); // start flywheel
  FW2.move(0.83 * 127 * reverseFW2);

  chassis.set_drive_pid(-9, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(40, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(22, 80, true);
  chassis.wait_drive();

  indexer.move(0);

  chassis.set_turn_pid(-270, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(35, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer);
  pros::delay(1100);

  indexer.move(-127 * reverseIndexer);

  FW1.move(0.82 * 127 * reverseFW1); // start flywheel
  FW2.move(0.82 * 127 * reverseFW2);

  chassis.set_turn_pid(-69, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(44, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(22, 80, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  indexer.move(0);

  chassis.set_drive_pid(35, DRIVE_SPEED, true);
  chassis.wait_drive();

  indexer.move(127 * reverseIndexer);
  pros::delay(1100);

  indexer.move(0);

  chassis.set_drive_pid(-50, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(53, TURN_SPEED);
  chassis.wait_drive();

  expansion.set_value(1);
}

void test() {
  back(DRIVE_SPEED);
}

/*void right_side() {
  chassis.set_drive_pid(27, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(18.5, TURN_SPEED);
  chassis.wait_drive();

  FW1.move(0.98 * 127 * reverseFW1); // start flywheel
  FW2.move(0.98 * 127 * reverseFW2);
  intake.move(127 * reverseIntake);

  pros::delay(2000);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(400);

  indexer.move(-indexerFeedSpeed * reverseIndexer);
  pros::delay(500);
  indexer.move(0);
  pros::delay(400);

  indexer.move(indexerFeedSpeed * reverseIndexer);
  pros::delay(500);

  FW1.move(0);
  FW2.move(0);
  indexer.move(60 * reverseIndexer);

  chassis.set_swing_pid(ez::LEFT_SWING, -55, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-28, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-10, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_drive_pid(3, DRIVE_SPEED, true);
  chassis.wait_drive();
}*/
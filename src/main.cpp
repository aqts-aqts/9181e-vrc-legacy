#include "main.h"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include <set>
using namespace global;

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-17, -9}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{5, 3}

  // IMU Port
  ,16

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.6666667

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

void initialize() {
  // Print our branding over your terminal :D
  expansionUpL.set_value(0);
  expansionDownL.set_value(0);
  expansionUpR.set_value(0);
  expansionDownR.set_value(0);
  init();
  odometry::init_odometry();
  ez::print_ez_template();
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Auto", left_side)
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

void disabled() {
  // . . .
}

void competition_initialize() {
  expansionUpL.set_value(0);
  expansionDownL.set_value(0);
  expansionUpR.set_value(0);
  expansionDownR.set_value(0);
}

void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  pros::Task positionTracking(odometry::positionTrack, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Position Tracking Task"); // Start position tracking task
  pros::Task flywheelControl(flywheelPID, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Control Task"); // Start flywheel control task
  // pros::Task discCounting(countDiscs, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Disc Counting Task"); // Start disc counting task

  startTime = pros::millis();
  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol() {
  use_pid = false;

  // set 1 - 0.766, set 2
  targetVelocity = 0.764; // current flywheel velocity
  lastTarget = 0.764; // last flywheel velocity

  int prevA = 0; // previous flywheel state change
  int prevPower = 0; // previous power change

  bool flywheel = true; // flywheel state

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  FW1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  FW2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  chassis.set_drive_current_limit(2500);

  pros::Task positionTracking(odometry::positionTrack, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Position Tracking Task"); // Start position tracking task
  pros::Task flywheelControl(flywheelPID, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel Control Task"); // Start flywheel control task
  // pros::Task discCounting(countDiscs, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Disc Counting Task"); // Start disc counting task

  use_pid = false;

  startTime = pros::millis();

  while (true) {
    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_standard(ez::SINGLE); // Flipped single arcade

    // Flywheel
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && elapsed - prevA > 500) {
    //   targetVelocity = (flywheel) ? 0.6: lastTarget;
    //   flywheel = !flywheel;
    //   crossed = false;
    //   prevA = elapsed;
    // }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && elapsed - prevA > 1000) {
      roll(80);
      prevA = elapsed;
    }

    // Control flywheel power
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && elapsed - prevPower > 100) { // flyPower = 0.805;
      targetVelocity = 0.95;
      lastTarget = 0.95;
      crossed = false;
      prevPower = elapsed;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && elapsed - prevPower > 100) { // flyPower = 0.75;
      targetVelocity = 0.764;
      lastTarget = 0.764;
      crossed = false;
      prevPower = elapsed;
    }

    // Aim
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) aim();

    // Intake/indexer
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // intake
      intake.move(intakeFeedSpeed * reverseIntake * ((discs > 3) ? -1: 1));
      indexer.move(indexerFeedSpeed * reverseIndexer);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // feed
      indexer.move(-indexerFeedSpeed * reverseIndexer);
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // stop
      intake.move(0);
      indexer.move(0);
    }

    // Rollers
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      intake.move(127 * reverseIntake); // Roll in
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // outtake
      intake.move(-127 * reverseIntake);
      // indexer.move(-127 * reverseIndexer);
    }

    // Limit drive current
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) chassis.set_drive_current_limit(2500);
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) chassis.set_drive_current_limit(1800);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && elapsed > 0) {
      expansionUpL.set_value(1);
      expansionUpR.set_value(1);
      expansionDownL.set_value(1);
      expansionDownR.set_value(1);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && elapsed > 0) {
      expansionDownL.set_value(1);
      expansionDownR.set_value(1);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && elapsed > 0) {
      expansionUpL.set_value(1);
      expansionDownL.set_value(1);
      expansionDownR.set_value(1);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && elapsed > 0) {
      expansionUpR.set_value(1);
      expansionDownL.set_value(1);
      expansionDownR.set_value(1);
    } // leave a bit of time before endgame as O(1) actions take non-zero time
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    elapsed += ez::util::DELAY_TIME;
  }
}
#include "main.h"
#include "pros/misc.h"
using namespace global;

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-8, -9}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{5, 3}

  // IMU Port
  ,6

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
  ,1.667

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
  expansion.set_value(1);
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
    Auton("Left side", programmingSkills),
    Auton("Right side", programmingSkills),
    Auton("Solo win point", programmingSkills)
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

void disabled() {
  // . . .
}

void competition_initialize() {
  expansion.set_value(1);
}

void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol() {
  double velocity = 560; // velocity of flywheel
  double lastVelocity = 560;
  double currentVelocity = 0;

  int prevFly = 0; // previous flywheel state change
  int prevPower = 0; // previous power change
  // int prevDisc = -1000; // last disc seen
  // double prevDist = indexer.get(); // previous distance to cup holder base

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  FW1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  FW2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  chassis.set_drive_current_limit(2500);

  while (true) {
    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    chassis.arcade_standard(ez::SINGLE); // Flipped single arcade

    odometry::updateSensors();
    odometry::updatePosition(); // update position tracking
    updateDisplay(); // update brain display

    currentVelocity += calculateFlywheelPower(velocity, 0.01, 0, 0);
    FW1.move_velocity(currentVelocity * reverseFW1);
    FW2.move_velocity(currentVelocity * reverseFW2);

    // Flywheel
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && elapsed - prevFly > 500) {
      velocity = (speeding) ? 200: lastVelocity;
      speeding = !speeding;
      prevFly = elapsed;
    }

    // Control flywheel power
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && elapsed - prevPower > 100) { // flyPower = 0.805;
      velocity = 560;
      lastVelocity = 560;
      prevPower = elapsed;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && elapsed - prevPower > 100) { // flyPower = 0.75;
      velocity = 420;
      lastVelocity = 420;
      prevPower = elapsed;
    }

    // Aim
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) aim();

    // Intake/indexer
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // intake
      intake.move(intakeFeedSpeed * reverseIntake);
      indexer.move(indexerFeedSpeed * reverseIndexer);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // feed
      indexer.move(-indexerFeedSpeed * reverseIndexer);
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // stop
      intake.move(0);
      indexer.move(0);
    }

    // Rollers
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      intake.move(-60 * reverseIntake); // Roll in
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // outtake
      intake.move(-127 * reverseIntake);
      // indexer.move(-127 * reverseIndexer);
    }

    // Limit drive current
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) chassis.set_drive_current_limit(2500);
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) chassis.set_drive_current_limit(1800);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && elapsed > 70000) expansion.set_value(1);
    // leave a bit of time before endgame as O(1) actions take non-zero time

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    elapsed += ez::util::DELAY_TIME; // increase elapsed time
  }
}
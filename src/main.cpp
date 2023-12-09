#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include <set>
using namespace global;

Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1, -2, -3} // non-skills/programming: -1, -2, -3; driver skills: -5, -6, -7

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{5, 6, 7} // non-skills/programming: 5, 6, 7; driver skills: 1, 2, 3

  // IMU Port
  ,8

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
  ,1.66667

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
  flaps.set_value(0);
  climber.set_value(0);
  init();

  // Print our branding over your terminal :D
  odometry::init_odometry();
  ez::print_ez_template();
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.06); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Auto", AWP)
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}

void disabled() {
  // . . .
}

void competition_initialize() {
  flaps.set_value(0);
  climber.set_value(0);
  // . . .
}

void pre_autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
}

void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  pros::Task positionTracking(odometry::positionTrack, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Position Tracking Task"); // Start position tracking task

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol() {
  bool flapState = false;
  bool clawState = false;
  bool climberState = false;

  int lastClimberChange = 0;
  int lastFlapChange = 0;
  int lastCatapultShot = 0;
  int lastClawChange = 0;

  chassis.set_drive_brake(MOTOR_BRAKE_BRAKE);
  pros::Task positionTracking(odometry::positionTrack, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Position Tracking Task"); // Start position tracking task

  while (true) {
    // chassis.tank(); // Tank control
    // chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_standard(ez::SINGLE); // Flipped single arcade

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && elapsed - lastCatapultShot > catapultCooldown) {
      lastCatapultShot = elapsed; // Set last catapult shot to current time
      activate_catapult(catapultRPM); // Activate catapult; put down to be able to pickup triball
    } 
    
    if (elapsed - lastCatapultShot > autoToManualCooldown) { // ONLY run manual control (don't allow if cooldown not over)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        catapult.move_velocity(catapultRPM); // Run catapult
      else
        catapult.move_velocity(0); // Stop
    }
    
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && elapsed - lastClimberChange > climberCooldown) {
      lastClimberChange = elapsed; // Set last climber change to current time
      climberState = !climberState; // Toggle climber state
      climber.set_value(climberState); // Set climber to state
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && elapsed - lastFlapChange > flapCooldown) {
      lastFlapChange = elapsed; // Set last flap change to current time
      flapState = !flapState; // Toggle flap state
      flaps.set_value(flapState); // Set flap to state
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && elapsed - lastClawChange > clawCooldown) {
      lastClawChange = elapsed; // Set last claw change to current time
      if (clawState) // If claw is up
        activate_claw(clawRPM * clawRaiseSpeed); // Activate claw
      else // If claw is down
        deactivate_claw(clawRPM * clawLowerSpeed); // Deactivate claw
      clawState = !clawState; // Toggle claw state
    } else if (elapsed - lastClawChange > clawDuration) { // ONLY run manual control (don't allow if cooldown not over)
      // Set claw to manual control
      claw.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * clawManualSpeed * reverseClaw);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
      claw.tare_position();
    
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    elapsed += ez::util::DELAY_TIME;
  }
}
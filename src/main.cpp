#include "main.h"
#include "lemlib/api.hpp"
using namespace global;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	left_motors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

	bool flap_state = false;
	bool lift_state = false;
	bool blocker_state = false;
	
	int last_flap_change = 0;
	int last_catapult_shot = 0;
	int last_lift_change = 0;

	int intake_time = 0;
	int intake_direction = 1;

	while (true) {
		drive();

		if (master.get_digital(DIGITAL_L1)) {
			if (elapsed - last_catapult_shot > CATAPULT_COOLDOWN) {
				rotate_catapult(CATAPULT_RPM);
				last_catapult_shot = elapsed;
			}
		}

		if (elapsed - last_catapult_shot > AUTO_TO_MANUAL_COOLDOWN) {
			if (master.get_digital(DIGITAL_L2))
				catapult.move_velocity(CATAPULT_RPM * REVERSE_CATAPULT);
			else
				catapult.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_B)) {
			if (elapsed - last_flap_change > FLAP_COOLDOWN) {
				flap_state = !flap_state;
				flaps.set_value(flap_state);
				last_flap_change = elapsed;
			}
		}

		if (master.get_digital(DIGITAL_R1)) {
			if (elapsed - last_lift_change > LIFT_COOLDOWN) {
				lift_state = !lift_state;
				if (lift_state) {
					intake_time = 2000;
					intake_direction = 1;
					if (lift.get_position() > LIFT_HOLD)
						raise_lower_lift(lift_state, 80);
					else
						raise_lower_lift(lift_state, LIFT_RPM);
				} else {
					lift.move_relative(200, LIFT_RPM * REVERSE_LIFT);
				}
				
				last_lift_change = elapsed;
			}
		}

		if (master.get_digital(DIGITAL_X)) {
			if (elapsed - last_lift_change > LIFT_COOLDOWN) {
				blocker_state = !blocker_state;
				lift_state = blocker_state;
				raise_lower_blocker(blocker_state, LIFT_RPM);
				last_lift_change = elapsed;
			}
		}

		if (master.get_digital(DIGITAL_R2)) {
			intake_time = 10;
			intake_direction = 1;
		} else if (master.get_digital(DIGITAL_Y)) {
			intake_time = 10;
			intake_direction = -1;
		}

		if (intake_time > 0) {
			intake.move_velocity(INTAKE_RPM * REVERSE_INTAKE * intake_direction);
			intake_time -= 10;
		} else {
			intake.move_velocity(0);
		}

		if (elapsed - last_lift_change > AUTO_TO_MANUAL_COOLDOWN) {
			lift.move_velocity(apply_threshold(master.get_analog(ANALOG_RIGHT_Y)));
		}

		pros::delay(10);
		elapsed += 10;
	}
}
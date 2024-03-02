#include "main.h"
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
void autonomous() {
	test();
}

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
	int last_lift_change = -100;
	int last_catapult_shot = 0;

	int intake_time = 0;
	int intake_direction = 1;

	while (true) {
		drive();

		/*if (master.get_digital(DIGITAL_L1)) {
			if (elapsed - last_catapult_shot > CATAPULT_COOLDOWN) {
				rotate_catapult(CATAPULT_RPM);
				last_catapult_shot = elapsed;
			}
		}*/

		if (elapsed - last_catapult_shot > AUTO_TO_MANUAL_COOLDOWN) {
			if (master.get_digital(DIGITAL_L2))
				catapult.move_velocity(CATAPULT_RPM * REVERSE_CATAPULT);
			else
				catapult.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_L1)) {
			if (elapsed - last_flap_change > FLAP_COOLDOWN) {
				flap_state = !flap_state;
				flaps.set_value(flap_state);
				last_flap_change = elapsed;
			}
		}

		if (master.get_digital(DIGITAL_R1)) {
			intake_time = 10;
			intake_direction = 1;
			if (elapsed - last_lift_change > LIFT_COOLDOWN)
				raise_lower_lift(true, LIFT_RPM);
			last_lift_change = elapsed;
		} else if (elapsed - last_lift_change == 10) {
			lift.move_relative(200, -LIFT_RPM);
			pros::Task([]() {
            	pros::delay(500);
            	raise_lower_lift(false, LIFT_RPM);
       		});
		}

		if (master.get_digital(DIGITAL_X)) {
			if (elapsed - last_lift_change > LIFT_COOLDOWN) {
				blocker_state = !blocker_state;
				raise_lower_blocker(blocker_state, LIFT_RPM);
				last_lift_change = elapsed;
			}
		}

		if (master.get_digital(DIGITAL_R2)) {
			lift.tare_position();
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

		// master.print(0, 0, "X: %f", chassis.getPose().x);
		// master.print(1, 0, "Y: %f", chassis.getPose().y);
		// master.print(2, 0, "Theta: %f", chassis.getPose().theta);

		double average_motor_temp = (left_motors.get_temperatures()[0] + left_motors.get_temperatures()[1] + left_motors.get_temperatures()[2] + right_motors.get_temperatures()[0] + right_motors.get_temperatures()[1] + right_motors.get_temperatures()[2]) / 6;
		master.print(0, 0, "Motor Temp: %f", average_motor_temp);

		pros::delay(10);
		elapsed += 10;
	}
}
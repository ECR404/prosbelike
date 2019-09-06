#include "main.h"

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

auto chassis = ChassisControllerFactory::create(
	2,
	-1,
	AbstractMotor::gearset::blue,
	{2_in, 12_in}
);

auto profileController = AsyncControllerFactory::motionProfile(
  10.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  chassis // Chassis Controller
);

void opcontrol() {
	profileController.generatePath({
		Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
		Point{2_ft, -2_ft, 0_deg}}, // The next point in the profile, 3 feet forward
		"A" // Profile name
	);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor right_mtr(1);
	pros::Motor left_mtr(2);
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;

		if(master.get_digital(DIGITAL_A))
		{
			// autonomous();
			profileController.setTarget("A");
			profileController.waitUntilSettled();
		}

		pros::lcd::set_text(3, std::to_string(-right_mtr.get_position()));
		pros::lcd::set_text(2, std::to_string(left_mtr.get_position()));

		pros::delay(20);
	}
}

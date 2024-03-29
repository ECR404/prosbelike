#include "main.h"

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
    pros::Motor left_mtr(2);
    pros::Motor right_mtr(1);
    left_mtr.move_relative(360, 100);
    right_mtr.move_relative(-360, 100);
    pros::delay(1000);
    left_mtr.move_relative(500, 100);
    pros::delay(1000);
    left_mtr.move_relative(360, 100);
    right_mtr.move_relative(-360, 100);
    pros::delay(1000);
}

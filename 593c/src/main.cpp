#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::green);
pros::MotorGroup leftMotors({4, 6, 7}, pros::MotorGearset::green);
pros::Controller master (CONTROLLER_MASTER);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "Oakdale 593C: Bearly Functioning");

}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while(true) {
		rightMotors.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));
		leftMotors.move(- master.get_analog(ANALOG_RIGHT_X) - master.get_analog(ANALOG_LEFT_Y));

	}
}
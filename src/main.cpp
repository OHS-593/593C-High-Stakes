#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup rightMotors ({1, 2, 3}, pros::MotorGearset::green);
pros::MotorGroup leftMotors ({-4, -6, -7}, pros::MotorGearset::green);
pros::adi::DigitalOut mogoClamp ('A');
pros::Controller masterCont (CONTROLLER_MASTER);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "Oakdale 593C: Bearly Functioning");
}

void driverDriver() {
	rightMotors.move(127);
	leftMotors.move(127);

	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while(true) {
		rightMotors.move_velocity(masterCont.get_analog(ANALOG_LEFT_Y) - masterCont.get_analog(ANALOG_RIGHT_X));
		leftMotors.move_velocity(masterCont.get_analog(ANALOG_LEFT_Y) + masterCont.get_analog(ANALOG_RIGHT_X));
	}
}

void driverClamp() {
	mogoClamp.set_value(false);
	
	while(true) {
		waitUntill(masterCont.get_digital_new_press(DIGITAL_R1));
		mogoClamp.set_value(true);
		waitUntill(!masterCont.get_digital_new_press(DIGITAL_R1));
		waitUntill(masterCont.get_digital_new_press(DIGITAL_R1));
		mogoClamp.set_value(false);
		waitUntill(!masterCont.get_digital_new_press(DIGITAL_R1));
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Task ws1(driverClamp);
	driverDriver();
}
#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::green); // right motors, green inserts
pros::MotorGroup leftMotors({-5, -6, -7}, pros::MotorGearset::green); // left motors, green inserts, reversed
pros::adi::DigitalOut mogoClamp('H');
pros::adi::DigitalOut sortClamp('G');
pros::Imu imu(19);
pros::Controller masterCont(CONTROLLER_MASTER);
pros::MotorGroup conveyor({-9, 10}, pros::MotorGearset::green); // conveyor motors, green inserts
pros::Motor walls(11, pros::MotorGearset::green); // wallstake motor
pros::Optical optical(15);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
							  &rightMotors, // right motor group
							  14.5, // track width
							  lemlib::Omniwheel::NEW_325, // drivetrain omni wheel size
							  333, // drivetrain rpm
							  2 // horizontal drift
);

// odometry sensor settings
lemlib::OdomSensors sensors(nullptr,nullptr,nullptr,nullptr,&imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                             3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
						lateral_controller,
						angular_controller,
						sensors
);

// print x,y,heading to brain/controller
void positionPrint() {
	while(true) {
		// print robot location to brain screen
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x pos
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y pos
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		// masterCont.print(0, 0, "X: %f", chassis.getPose().x);
		pros::delay(20);
	}
}

// driver control, control drivetrain
void driverDriver() {
	rightMotors.move(200);
	leftMotors.move(200);

	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

	while(true) {
		rightMotors.move_velocity((masterCont.get_analog(ANALOG_LEFT_Y)*1.575) - (masterCont.get_analog(ANALOG_RIGHT_X)*1.575));
		leftMotors.move_velocity((masterCont.get_analog(ANALOG_LEFT_Y)*1.575) + (masterCont.get_analog(ANALOG_RIGHT_X)*1.575));
		pros::delay(2);
	}
}

// driver control, control clamp
void driverClamp() {	
	while(true) {
		waitUntill(masterCont.get_digital_new_press(DIGITAL_L1));
		mogoClamp.set_value(true);
		waitUntill(masterCont.get_digital_new_press(DIGITAL_L1));
		mogoClamp.set_value(false);
		pros::delay(2);
	}
}

void driverSort() {	
	while(true) {
		waitUntill(masterCont.get_digital_new_press(DIGITAL_L2));
		sortClamp.set_value(true);
		waitUntill(masterCont.get_digital_new_press(DIGITAL_L2));
		sortClamp.set_value(false);
		pros::delay(2);
	}
}

// driver control, control intake / conveyor
void driverHook() {
	conveyor.move(200);

	while(true) {
		if(masterCont.get_digital(DIGITAL_R1)) {
			conveyor.move_velocity(200);
			pros::delay(2);
		} else if(masterCont.get_digital(DIGITAL_R2)) {
			conveyor.move_velocity(-200);
			pros::delay(2);
		}
		else {
			conveyor.move_velocity(0);
			pros::delay(2);
		}
	}
}

// driver control, control wallstakes
void driverWall() {
	walls.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	walls.move(200);

	while(true) {
		if(masterCont.get_digital(DIGITAL_B)) {
			walls.move_velocity(100);
			pros::delay(2);
		} else if(masterCont.get_digital(DIGITAL_DOWN)) {
			walls.move_velocity(-100);
			pros::delay(2);
		/*} else if(masterCont.get_digital(DIGITAL_RIGHT)) {
			walls.move_relative(-0.1, 100);
			if (walls.get_position() < 1) {
				walls.move_velocity(-100);
			}
			else {
				walls.move_velocity(100);
			}
			while (!((walls.get_position() < 10) && (walls.get_position() > -10))) {
				pros::delay(2);
			}
			pros::delay(2);*/
		} else if(masterCont.get_digital(DIGITAL_Y)) {
			walls.move_relative(0, 100);
			walls.move_velocity(100);
			while (!((walls.get_position() < 10) && (walls.get_position() > -10))) {
				pros::delay(2);
			}
			pros::delay(2);
		}
		else {
			walls.move_velocity(0);
			pros::delay(2);
		}
	}
}

// auton drive controlls
void autonDrive(float he, float ie, int jv) {
	leftMotors.tare_position_all();
	rightMotors.tare_position_all();

	float hd = (he / 360);
	float id = (ie / 360);

	pros::lcd::print(4, "id: %f", id);
	pros::lcd::print(5, "ie: %f", ie);

	rightMotors.move_relative(id, jv);
	leftMotors.move_relative(hd, jv);
	// wait until motors reach pos
	while (!((rightMotors.get_position() < id+0.1) && (rightMotors.get_position() > id-0.1))) {
		pros::delay(2);
	}
}

void autonLeft() {
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	autonDrive(-700, -700, 50);
	mogoClamp.set_value(true);
	pros::delay(1000);
	conveyor.move_relative(2000, 200);
	pros::delay(1000);
	conveyor.move_relative(20000, 200);
	autonDrive(250, -250, 50);
	// autonDrive(330, -330, 50);
	autonDrive(250, 250, 50);
	autonDrive(-600, 600, 50);
	autonDrive(700, 700, 50);
}

void autonRight() {
	rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	autonDrive(-700, -700, 50);
	mogoClamp.set_value(true);
	pros::delay(1000);
	conveyor.move_relative(2000, 200);
	pros::delay(1000);
	conveyor.move_relative(20000, 200);
	autonDrive(-250, 250, 50);
	// autonDrive(-330, 330, 50);
	autonDrive(250, 250, 50);
	autonDrive(600, -600, 50);
	autonDrive(700, 700, 50);
}

void autonSkills() {
	chassis.moveToPoint(0, 0, 5000);
	chassis.moveToPoint(0.168, 25.207, 5000);
	chassis.moveToPoint(24.963, 27.193, 5000);
	chassis.moveToPoint(51.576, 28.53, 5000);
	chassis.moveToPoint(-0.139, 24.842, 5000);
	chassis.moveToPoint(-36.07, -7.74, 5000);
	chassis.moveToPoint(-51.912, 27.393, 5000);
	chassis.moveToPoint(-61.373, 64.778, 5000);
	chassis.moveToPoint(-49.374, 100.166, 5000);
	chassis.moveToPoint(-5.266, 125.296, 5000);
	chassis.moveToPoint(-89.87, 77.093, 5000);
}

// initalize function, runs on program startup
void initialize() {
	pros::lcd::initialize(); // initalize brain screen
	chassis.calibrate(); // calibrate sensors
	walls.tare_position(); // tare position of wallstake mech

	optical.set_led_pwm(100);

	// print position to brain
	pros::Task ws1(positionPrint);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	autonLeft();
}

void opcontrol() {
	pros::Task ws1(driverClamp);
	pros::Task ws2(driverSort);
	pros::Task ws3(driverHook);
	pros::Task ws4(driverWall);
	driverDriver();
}
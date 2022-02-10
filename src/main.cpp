#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::ADIDigitalOut claw('h');
	claw.set_value(HIGH);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::ADIDigitalOut claw('h');
	claw.set_value(HIGH);
}

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
	pros::Motor lLift(19, 1);
	pros::Motor rLift(20);
	pros::Motor lf(16, 1);
	pros::Motor rf(13);
	pros::Motor lr(17, 1);
	pros::Motor rr(14);
	pros::ADIDigitalOut claw('h');

	pros::ADIDigitalOut stab('g');

	claw.set_value(HIGH);
	stab.set_value(LOW);

	lLift.move_velocity(-100);
	rLift.move_velocity(-100);
	pros::delay(5000);
	lLift.move_velocity(0);
	rLift.move_velocity(0);
	lf.move_velocity(50);
	rf.move_velocity(50);
	lr.move_velocity(50);
	rr.move_velocity(50);
	stab.set_value(HIGH);

	pros::delay(1000);
	lf.move_velocity(0);
	rf.move_velocity(0);
	lr.move_velocity(0);
	rr.move_velocity(0);
	claw.set_value(LOW);


	lf.move_velocity(10);
	rf.move_velocity(10);
	lr.move_velocity(10);
	rr.move_velocity(10);

	pros::delay(500);
	lf.move_velocity(0);
	rf.move_velocity(0);
	lr.move_velocity(0);
	rr.move_velocity(0);

	lf.move_velocity(-50);
	rf.move_velocity(-50);
	lr.move_velocity(-50);
	rr.move_velocity(-50);

	pros::delay(1500);
	lf.move_velocity(0);
	rf.move_velocity(0);
	lr.move_velocity(0);
	rr.move_velocity(0);
	stab.set_value(LOW);
	lLift.move_velocity(-50);
	rLift.move_velocity(-50);
	pros::delay(2000);
	lLift.move_velocity(0);
	rLift.move_velocity(0);
/*
	lf.move_velocity(100);
	rf.move_velocity(100);
	lr.move_velocity(100);
	rr.move_velocity(100);

	pros::delay(8000);
	lf.move_velocity(0);
	rf.move_velocity(0);
	lr.move_velocity(0);
	rr.move_velocity(0);*/






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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor lf(16, 1);
	pros::Motor rf(13);
	pros::Motor lr(17, 1);
	pros::Motor rr(14);
	pros::Motor lLift(19, 1);
	pros::Motor rLift(20);
	pros::Rotation r(1);
	pros::ADIDigitalOut stab('g');
	pros::ADIDigitalOut claw('h');

	lLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	int rDrive;
	int lDrive;
	//r.reset_position();
	lLift.set_zero_position(lLift.get_position());
	int liftPos = r.get_position();
	claw.set_value(HIGH);

	while (true) {
		//drivetrain
		rDrive = master.get_analog(ANALOG_LEFT_Y);
		lDrive = master.get_analog(ANALOG_RIGHT_Y);
		lf.move(lDrive);
		rf.move(rDrive);
		lr.move(lDrive);
		rr.move(rDrive);

		//__arm
		//printf("Angle: %i \n", r.get_position());
		master.clear();
		master.print(0, 0,"Angle: %d", r.get_position());

			if (master.get_digital(DIGITAL_R1)) {
				//move up
				rLift.move_velocity(170);
				lLift.move_velocity(170);
			}
			 else if (master.get_digital(DIGITAL_R2)) {
				//move up
				rLift.move_velocity(-170);
				lLift.move_velocity(-170);
			}
			else {
				//move up
				rLift.move_velocity(0);
				lLift.move_velocity(0);
			}

		if (master.get_digital(DIGITAL_A)) {
			stab.set_value(HIGH);
		}
		else if (master.get_digital(DIGITAL_B)) {
			stab.set_value(LOW);
		}

		//claw
		if (master.get_digital(DIGITAL_L1)) {
			claw.set_value(HIGH);
		}
		else if (master.get_digital(DIGITAL_L2)) {
			claw.set_value(LOW);
		}

		pros::delay(20);
	}
}

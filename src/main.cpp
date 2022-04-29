#include "main.h"
#include "PID/PID.h"
using namespace okapi;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

#define P 0.8
#define I 1
#define D 0.05


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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor lf(16, 1);
	pros::Motor rf(13);
	pros::Motor lr(17, 1);
	pros::Motor rr(14);
	pros::Motor lLift(19, 1);
	pros::Motor rLift(20);
	pros::Rotation rot(3);
	pros::Distance dist(10);
	pros::Distance distFront(6);
	pros::ADIDigitalOut stab('g');
	pros::ADIDigitalOut claw('h');
	pros::ADIDigitalIn goal('f');
	pros::ADIDigitalOut quick('e');
	pros::ADIDigitalIn wall('c');
	lLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//pros::delay(20);
	//claw.set_value(HIGH);
	//stab.set_value(LOW);

	const double liftkP = 0.001;
	const double liftkI = 0.0001;
	const double liftkD = 0.0001;

	int select = 4; //0-comp,1-skills,2-test,3-test-comp
	//comp auton
	if (select == 0) {

		lLift.move_velocity(-100);
		rLift.move_velocity(-100);
		pros::delay(1700);
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

		pros::delay(1000);
		lf.move_velocity(0);
		rf.move_velocity(0);
		lr.move_velocity(0);
		rr.move_velocity(0);
		stab.set_value(LOW);
		lLift.move_velocity(-30);
		rLift.move_velocity(-30);
		pros::delay(2000);
		lLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		rLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		lLift.move_velocity(30);
		rLift.move_velocity(30);
		pros::delay(800);
		lLift.move_velocity(0);
		rLift.move_velocity(0);
		lf.move_velocity(50);
		rf.move_velocity(50);
		lr.move_velocity(50);
		rr.move_velocity(50);


		pros::delay(1500);

		claw.set_value(HIGH);
		pros::delay(500);
		lf.move_velocity(-50);
		rf.move_velocity(-50);
		lr.move_velocity(-50);
		rr.move_velocity(-50);

		lLift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rLift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		pros::delay(2500);
		lf.move_velocity(0);
		rf.move_velocity(0);
		lr.move_velocity(0);
		rr.move_velocity(0);

	}

//skills auton
	else if (select == 1) {
		std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors({16, 17}, {13, 14}) //{lF, lR}, {rF, rR}	{16, 17}, {13, 14}
			.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 10.875_in}, imev5GreenTPR})
			.withOdometry()
			.withMaxVelocity(50)
			.withMaxVoltage(6000)
			.buildOdometry();
		std::shared_ptr<AsyncPositionController<double, double>> liftController =
			AsyncPosControllerBuilder()
				.withMotor({-19,-20})
				.withMaxVelocity(140)
				//.withGains({liftkP, liftkI, liftkD})
				.withSensor(3)
				.build();
      pros::delay(50);
			stab.set_value(HIGH);
			//chassis->moveDistance(6_in);
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			chassis->moveDistanceAsync(20_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			chassis->waitUntilSettled();
			liftController->setTarget(1600);
			liftController->waitUntilSettled();
			chassis->setMaxVelocity(80);
			chassis->moveDistance(-10_in);
			chassis->turnAngle(90_deg);
			chassis->moveDistance(20_in);
			chassis->turnAngle(92_deg);
			chassis->moveDistance(-70_in);
			chassis->moveDistance(10_in);
			chassis->turnAngle(-90_deg);
			chassis->moveDistance(30_in);
			chassis->turnAngle(-75_deg);
			chassis->moveDistance(8_in);
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			claw.set_value(LOW);
			//chassis->moveDistance(-12_in);
			chassis->turnAngle(145_deg);
			chassis->moveDistance(50_in);
			/*
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			claw.set_value(LOW);
			chassis->moveDistance(-18_in);
			liftController->setTarget(1600);
			chassis->turnAngle(-90_deg);

			//drive
			chassis->moveDistance(24_in);

			//turn
			chassis->turnAngle(90_deg);

			//move lift down
			liftController->setTarget(4300);
			liftController->waitUntilSettled();

			//drive and grab goal
			chassis->moveDistanceAsync(14_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			chassis->waitUntilSettled();

			//bring lift up
			liftController->setTarget(1600);
			liftController->waitUntilSettled();
*/


			//liftController->setTarget(2400);
			//claw.set_value(LOW);
			//chassis->moveDistance(-15_in);



	}
	else if (select == 2) {
		std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors({16, 17}, {13, 14}) //{lF, lR}, {rF, rR}	{16, 17}, {13, 14}
			.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 10.875_in}, imev5GreenTPR})
			.withOdometry()
			.withMaxVelocity(80)
			.withMaxVoltage(10000)
			.buildOdometry();
		std::shared_ptr<AsyncPositionController<double, double>> liftController =
			AsyncPosControllerBuilder()
				.withMotor({-19,-20})
				//.withMaxVelocity(80)
				.withGains({liftkP, liftkI, liftkD})
				.withSensor(3)
				.build();
      pros::delay(50);
			stab.set_value(HIGH);
			chassis->moveDistanceAsync(50_in);
			liftController->setTarget(4300);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			chassis->stop();
			liftController->setTarget(1600);
			liftController->waitUntilSettled();
			chassis->setMaxVelocity(80);
			chassis->moveDistance(40_in);
			chassis->turnAngle(90_deg);
			chassis->moveDistance(40_in);
			chassis->turnAngle(-80_deg);
			chassis->moveDistance(8_in);
			liftController->setTarget(2400);
			claw.set_value(LOW);
			chassis->moveDistance(-15_in);




			//pros::lcd::clear();
			//pros::lcd::print(0,"stopped");
	}
	else if (select == 3) {
		std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors({16, 17}, {13, 14}) //{lF, lR}, {rF, rR}	{16, 17}, {13, 14}
			.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 10.75_in}, imev5GreenTPR})
			.withOdometry()
			.withMaxVelocity(195)
			.withMaxVoltage(11500)
			.buildOdometry();
		std::shared_ptr<AsyncPositionController<double, double>> liftController =
			AsyncPosControllerBuilder()
				.withMotor({-19,-20})
				.withMaxVelocity(140)
				//.withGains({liftkP, liftkI, liftkD})
				.withSensor(3)
				.build();
      pros::delay(50);
			stab.set_value(LOW);

			//liftController->setTarget(2400);
			chassis->moveDistanceAsync(-70_in);
			while (dist.get() > 230) {
				pros::delay(10);
			}
			quick.set_value(HIGH);
			chassis->stop();
			chassis->moveDistanceAsync(70_in);
			for (int i = 0; i < 125; i++) {
				pros::delay(20);
			}
			//pros::delay(50);
			chassis->stop();
			quick.set_value(LOW);
			chassis->setMaxVelocity(50);
			chassis->moveDistance(-16_in);
			chassis->turnAngle(90_deg);
			chassis->moveDistanceAsync(30_in);
			while (distFront.get() > 250) {
				pros::delay(20);
			}
			chassis->stop();
			chassis->turnAngle(90_deg);
			chassis->moveDistance(18_in);
			liftController->setTarget(2400);
			liftController->waitUntilSettled();
			pros::delay(200);
			liftController->setTarget(1600);
			//dump

			chassis->moveDistance(-4_in);
			chassis->turnAngle(-45_deg);
			chassis->moveDistance(-20_in);
			chassis->turnAngle(75_deg);
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			stab.set_value(HIGH);
			pros::delay(200);
			chassis->moveDistanceAsync(16_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			//chassis->stop();
			liftController->setTarget(1600);
			liftController->waitUntilSettled();
			chassis->waitUntilSettled();

	}
	else if (select == 4) {
		std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors({16, 17}, {13, 14}) //{lF, lR}, {rF, rR}	{16, 17}, {13, 14}
			.withDimensions(AbstractMotor::gearset::green, {{3.75_in, 10.8_in}, imev5GreenTPR})
			.withOdometry()
			.withMaxVelocity(198)
			.withMaxVoltage(12000)
			.buildOdometry();
		std::shared_ptr<AsyncPositionController<double, double>> liftController =
			AsyncPosControllerBuilder()
				.withMotor({-19,-20})
				.withMaxVelocity(140)
				//.withGains({liftkP, liftkI, liftkD})
				.withSensor(3)
				.build();
      pros::delay(50);
			stab.set_value(LOW);

			//liftController->setTarget(2400);
			chassis->moveDistanceAsync(-70_in);
			pros::delay(300);
			while (dist.get() > 500) {
				pros::delay(10);
			}
			quick.set_value(HIGH);
			chassis->stop();
			chassis->moveDistanceAsync(60_in);
			int i = 0;
			while (wall.get_value() == HIGH && i < 125) {
				pros::delay(20);
				i++;
			}
			//pros::delay(50);
			chassis->stop();

			chassis->setMaxVelocity(50);
			chassis->moveDistance(-16_in);
			quick.set_value(LOW);
			pros::delay(300);
			chassis->turnAngle(90_deg);
			chassis->moveDistanceAsync(30_in);
			while (distFront.get() > 640) {
				pros::delay(20);
			}
			chassis->stop();
			chassis->turnAngle(55_deg);

			chassis->moveDistance(20_in);
			liftController->setTarget(2400);
			liftController->waitUntilSettled();
			pros::delay(200);
			liftController->setTarget(1600);
			//dump

			chassis->moveDistance(-10_in);
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			stab.set_value(HIGH);
			pros::delay(200);
			chassis->moveDistanceAsync(12_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			chassis->waitUntilSettled();
			chassis->moveDistance(-20_in);
			chassis->waitUntilSettled();
			chassis->turnAngle(-55_deg);
			claw.set_value(LOW);
			chassis->turnAngle(95_deg);
			chassis->waitUntilSettled();
			//chassis->moveDistance(-4_in);
			//chassis->turnAngle(25_deg);
			chassis->moveDistanceAsync(20_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			chassis->waitUntilSettled();
			/*
			chassis->turnAngle(-90_deg);
			chassis->moveDistance(-48_in);
			chassis->moveDistance(14_in);
			chassis->turnAngle(120_deg);
			liftController->setTarget(4300);
			liftController->waitUntilSettled();
			stab.set_value(HIGH);
			pros::delay(200);
			chassis->moveDistanceAsync(46_in);
			while (goal.get_value() == 0) {
				pros::delay(20);
			}
			claw.set_value(HIGH);
			//chassis->stop();
			liftController->setTarget(1600);
			liftController->waitUntilSettled();
			chassis->waitUntilSettled();
			chassis->moveDistance(-48_in);
			*/
	}
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
	pros::Rotation rot(3);
	pros::Distance dist(10);
	pros::ADIDigitalOut stab('g');
	pros::ADIDigitalOut claw('h');
	pros::ADIDigitalIn goal('f');
	pros::ADIDigitalOut quick('e');
	pros::ADIDigitalIn armLimit('d');
	pros::ADIDigitalIn wall('c');
	lLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	int rDrive;
	int lDrive;

  bool toLimit = false;
	bool toTarget = false;
	int armTarget = 700;
	int armSpeed = 0;

	//r.reset_position();
	//lLift.set_zero_position(lLift.get_position());
	//int liftPos;
	claw.set_value(HIGH);
	//r.reset();
	int count = 0;
	rLift.tare_position();

	bool grab =  false;
	bool ready = false;
	bool wallMode = false;

	//rot.reset();
	rot.set_reversed(true);

	while (true) {
		//pros::lcd::clear();
		//pros::lcd::print(0, "ArmPos: %d", rLift.get_position());

		//drivetrain
		rDrive = master.get_analog(ANALOG_LEFT_Y);
		lDrive = master.get_analog(ANALOG_RIGHT_Y);
		lf.move(lDrive);
		rf.move(rDrive);
		lr.move(lDrive);
		rr.move(rDrive);

		//arm

			if (master.get_digital(DIGITAL_R1)) {
				//move up
				rLift.move_velocity(180);
				lLift.move_velocity(180);
				toTarget = false;
				toLimit = false;
			}
			else if (master.get_digital(DIGITAL_R2) && rot.get_angle() > 15000 && rot.get_angle() < 17000) {
				rLift.move_velocity(-180);
	 			lLift.move_velocity(-180);
				toTarget = false;
				toLimit = false;
			}
			else if (armLimit.get_value() == LOW && master.get_digital(DIGITAL_R2)) {
				rLift.move_velocity(-180);
	 			lLift.move_velocity(-180);
				toTarget = false;
				toLimit = false;
			}
			else {
				//move up
				rLift.move_velocity(0);
				lLift.move_velocity(0);
			}



		if (master.get_digital(DIGITAL_A)) {
			stab.set_value(HIGH);//out
		}
		else if (master.get_digital(DIGITAL_B)) {
			stab.set_value(LOW);//in
		}

		if (master.get_digital(DIGITAL_X)) {
			//if (dist.get() < 180) {
				quick.set_value(HIGH);//out
			//}
		}
		else if (master.get_digital(DIGITAL_Y)) {
			quick.set_value(LOW);//in
		}

		//claw
		if (master.get_digital(DIGITAL_L1)) { //close
			//if (goal.get_value()== 1) {
				//grab = true;
				claw.set_value(HIGH);

		}
		else if (master.get_digital(DIGITAL_L2)) { //open
			claw.set_value(LOW);
			ready = true;
		}

		if (ready) {
			if (goal.get_new_press()) {
				ready = false;
				claw.set_value(HIGH);
			}
		}
		/*if(grab) {
			if (goal.get_value() == 1) {
				claw.set_value(HIGH);
				grab = false;
			}
		}*/

		//if (master.get_digital(DIGITAL_LEFT)) {
		//	rot.set_position(0);
		//	rot.set_reversed(true);
		//}

		if (master.get_digital(DIGITAL_DOWN)) {
			toLimit = true;
		}
		if (master.get_digital(DIGITAL_UP)) {
			armTarget = 12000;
			toTarget = true;
		}

		if (wall.get_value()) {
			armTarget = 19000;
			toTarget = true;
			wallMode = true;
		}

		if (wallMode == true) {
			if (lDrive < 0 || rDrive < 0) {
				armTarget = 12000;
				toTarget = true;
				wallMode = false;
			}
		}


		if (toLimit) {
			if (armLimit.get_value() == HIGH) {
				toLimit = false;
				rot.set_position(25000);
			}
			else {
				rLift.move_velocity(-120);
	 			lLift.move_velocity(-120);
			}
		}
		else if (toTarget){
			if (rot.get_position() > armTarget + 400){armSpeed = 120;}
			else if (rot.get_position() < armTarget - 400){armSpeed = -120;}
			else {toTarget = false;armSpeed = 0;}
			rLift.move_velocity(armSpeed);
	 		lLift.move_velocity(armSpeed);
		}
		else{}

		int pidSource = rot.get_position();


		std::cout << toTarget << std::endl;
		std::cout << wall.get_value() << std::endl;
		pros::delay(20);


	}
}

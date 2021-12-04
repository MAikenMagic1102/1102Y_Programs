#include "main.h"

//Degrees
int PosBaseOpen = 8;
int PosBaseClose = 93;

int PosArmUP = 27;
int PosArmDown = 92;
int PosArmBridge = 53;
int PosArmBowl = 75;

int PincherOpen = 290;
int PincherClose = 410;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	//Robot Part Defines
	//Drivetrain Definitions
		std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
				.withMotors({19,20}, {-11,-12}) // Minus sign means motors are reversed
				.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
			  .withOdometry()
				.build();

	//Base Latch Definition
		RotationSensor baseLatchSensor(9);
		auto bS = std::make_shared<RotationSensor>(baseLatchSensor);

		const double baseLatchkP = 0.02;
		const double baseLatchkI = 0.0001;
		const double baseLatchkD = 0.0001;

		std::shared_ptr<AsyncPositionController<double, double>> baseLatch =
			AsyncPosControllerBuilder()
				.withMotor(3)
				.withGains({baseLatchkP, baseLatchkI, baseLatchkD})
				.withSensor(bS)
				.build();

		baseLatch->setTarget(5);

		drive->setMaxVelocity(80);
		drive->moveDistance(-24_in);

		while(PosBaseClose - baseLatchSensor.get() > 15){
			baseLatch->setTarget(PosBaseClose);
		}
		drive->turnAngle(-128_deg);
		drive->moveDistance(-20_in);

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
	Controller driver_controller;
	Controller operator_controller;

	ControllerButton ArmUp(ControllerId::partner,ControllerDigital::up);
	ControllerButton ArmDown(ControllerId::partner,ControllerDigital::down);
	ControllerButton ArmBridge(ControllerId::partner, ControllerDigital::left);

	ControllerButton BaseLatchOpen(ControllerId::partner,ControllerDigital::L1);
	ControllerButton BaseLatchClose(ControllerId::partner,ControllerDigital::L2);

	ControllerButton ArmLatchOpen(ControllerDigital::L1);
	ControllerButton ArmLatchClose(ControllerDigital::L2);

	ControllerButton ArmManualUp(ControllerDigital::R1);
	ControllerButton ArmManualDown(ControllerDigital::R2);

//Robot Part Defines
//Drivetrain Definitions
	std::shared_ptr<ChassisController> drive =
	ChassisControllerBuilder()
			.withMotors({19,20}, {-11,-12}) // Minus sign means motors are reversed
			.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
			.build();

//Arm Defines
	RotationSensor armSensor(6);
	auto aS = std::make_shared<RotationSensor>(armSensor);

	const double armKP = 0.035;
	const double armkI = 0.0001;
	const double armkD = 0.001;

	std::shared_ptr<AsyncPositionController<double, double>> arm =
		AsyncPosControllerBuilder()
			.withMotor({7, -1})
			.withGains({armKP, armkI, armkD})
			.withSensor(aS)
			.build();

			//Base Latch Definition
	RotationSensor baseLatchSensor(9);
	auto bS = std::make_shared<RotationSensor>(baseLatchSensor);

	const double baseLatchkP = 0.02;
	const double baseLatchkI = 0.0001;
	const double baseLatchkD = 0.0001;

	std::shared_ptr<AsyncPositionController<double, double>> baseLatch =
		AsyncPosControllerBuilder()
			.withMotor(3)
			.withGains({baseLatchkP, baseLatchkI, baseLatchkD})
			.withSensor(bS)
			.build();

	Motor pincher(4, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
	pincher.tarePosition();

	baseLatch->setTarget(PosBaseClose);
	arm->setTarget(PosArmBridge);
	double lastPosition = PosArmBridge;
	pros::delay(100);
	arm->setTarget(PosArmDown);
  lastPosition = PosArmDown;

	while (true) {
		drive->getModel()->arcade(driver_controller.getAnalog(ControllerAnalog::leftY),
										driver_controller.getAnalog(ControllerAnalog::rightX));

	  //Base CAM Pincher Control
	  if(BaseLatchOpen.changedToPressed()){
			baseLatch->setTarget(PosBaseOpen);
		}else{
			if(BaseLatchClose.changedToPressed()){
				baseLatch->setTarget(PosBaseClose);
			}
		}

		if(ArmLatchOpen.changedToPressed()){
			pincher.moveAbsolute(PincherOpen, 200);
		}else{
			if(ArmLatchClose.changedToPressed()){
				pincher.moveAbsolute(PincherClose, 200);
			}
		}

		//Arm Control
if(ArmUp.changedToPressed()){
	arm->setTarget(PosArmUP);
	lastPosition = PosArmUP;
}else{
	if(ArmDown.changedToPressed()){
		arm->setTarget(PosArmDown);
		lastPosition = PosArmDown;
	}else{
		if(ArmBridge.changedToPressed()){
			arm->setTarget(PosArmBridge);
			lastPosition = PosArmBridge;
		}else{
			if(ArmManualUp.isPressed()){
				arm->setTarget(armSensor.get() - 25);
				lastPosition = armSensor.get();
			}else{
				if(ArmManualDown.isPressed()){
					arm->setTarget(armSensor.get() + 25);
					lastPosition = armSensor.get();
				}else{
					arm->setTarget(lastPosition);
				}
			}
		}
	}
}

	}
}

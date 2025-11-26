#include "main.h"
#include "config.h"
#include "lemlib/api.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  11,						  // 12 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);
	::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_325,0);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
							nullptr,				  // vertical tracking wheel 2, set to nullptr as we are using IMEs
							nullptr,				  // horizontal tracking wheel 1
							nullptr,				  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu					  // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(6,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  24,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  15   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(5.8, // proportional gain (kP)3.2  5.8
											  0,   // integral gain (kI)
											  39.5,  // derivative gain (kD)28 39.5
											  0,   // anti windup
											  0,   // small error range, in degrees
											  0,   // small error range timeout, in milliseconds
											  0,   // large error range, in degrees
											  0,   // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
	steer_curve(3,	  // joystick deadband out of 127
				10,	  // minimum output where drivetrain will move out of 127
				1.019 // expo curve gain0
	);

lemlib::ExpoDriveCurve
	throttle_curve(3,	 // joystick deadband out of 127
				   10,	 // minimum output where drivetrain will move out of 127
				   1.019 // expo curve gain
	);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
											// &throttle_curve, &steer_curve
);

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	pros::Task screen_task([&]()
						   {
        while (true) {
			pros::screen::print(TEXT_MEDIUM, 0, "X: %f", chassis.getPose().x);
			pros::screen::print(TEXT_MEDIUM, 1, "Y: %f", chassis.getPose().y);
			pros::screen::print(TEXT_MEDIUM, 2, "Theta: %f", chassis.getPose().theta);

			//pros::screen::print(TEXT_MEDIUM, 3, "IMU: %f", imu.get_heading());

			pros::delay(20);
        } });
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







//AUTON CODES

void pidforwardTune()
{
	while (true)
	{
		
		 chassis.moveToPoint(0,48, 4000);
		 pros::delay(4000);
		 chassis.moveToPoint(0,48, 4000, {.forwards = false});
		 pros::delay(4000);
	}
}
void pidTurnTune(){
	while (true)
	{
		chassis.turnToHeading(90, 1000);
		pros::delay(2000);
		chassis.turnToHeading(0, 1000);
		pros::delay(2000);
		// chassis.moveToPoint(0,48, 4000);
		// pros::delay(4000);
		// chassis.moveToPoint(0,0, 4000, {.forwards = false});
		// pros::delay(4000);
	}
}

void autonomous() {
	pidforwardTune();
}




//intake FUNCTIONS
void intakeHighgoal(){
	Intake_High_mg.move(127);
	
}

void intakeMiddlegoal(){
	Intake_Middle_mg.move(127);
	
}

void IntakeReverse(){
	Intake_High_mg.move(-127);
}
void IntakeStop(){
	Intake_High_mg.move(0);
	Intake_Middle_mg.move(0);
}
void IntakeSrore(){
	Intake_Shooter.move(127);
	Intake_Shooter_Beaver.move(60);
}
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);
		            
		// Sets right motor voltage
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			Intake_High_mg.move(-127);
		}
		//reverse high goal
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			Intake_High_mg.move(127);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			Intake_Middle_mg.move(-127);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			IntakeSrore();
		}
		else{
			Intake_High_mg.move(0);
			Intake_Middle_mg.move(0);
		}
		pros::delay(20);  
		                             // Run for 20 ms then update
	}

}
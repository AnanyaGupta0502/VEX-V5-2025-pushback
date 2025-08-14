#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

//MOVEMENTS
// chassis.MoveToPose() Moves the robot to a specific position (x, y) on the field and ends at a specific heading (orientation).The robot will follow a smooth, curved path if the heading is different from the current heading.
// chassis.turnToHeading() Turns the robot in place to a specific heading (angle in degrees), without changing its position.
// chassis.moveToPoint() Moves the robot to a specific point (x, y) on the field without changing its heading. The robot will follow a straight line path.
// chassis.turnToPoint() Turns the robot to face a specific point (x, y) on the field without changing its position.
// chassis.follow() Follows a predefined path with a specified lookahead distance and timeout. The robot will follow the path smoothly, adjusting its heading as needed.

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//L1 is for spinning all motors to the left (some motors are reversed), L2 is for bottom motors left, and top 2 ones are right, R1 is for all spinning right

// motor groups
pros::MotorGroup right_motors({-15, 16, 20}, pros::MotorGearset::blue ); // left motors on ports
pros::MotorGroup left_motors({13, -11, -12}, pros::MotorGearset::blue ); // right motors on ports (removed negative signs)
pros::Motor intake_top(10, pros::MotorGearset::blue);
pros::Motor intake_bottom(14, pros::MotorGearset::blue);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
	&right_motors, // right motor group
    11.375, // 11.375 inch track width
    lemlib::Omniwheel::NEW_4, // using new 4" omnis
	343, // drivetrain rpm is 450 <- use this to change the speed of the robot
	2 // horizontal drift is 2 (for now)
);

// Inertial Sensor on port 10
pros::Imu imu(9);


// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP) can be moved to 3 or 4 (3.5)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD) can be moved to 6 or 8 (7)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(1);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
/**Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                OdomSensors sensors, DriveCurve* throttleCurve = &defaultDriveCurve,
                DriveCurve* steerCurve = &defaultDriveCurve); */
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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

//code for controller and intake control 
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Debug: Print controller values to console
        printf("Controller - LeftY: %d, RightX: %d\n", leftY, rightX);

        // move the robot
        // prioritize steering slightly
        chassis.arcade(leftY, rightX, false, 0.75);

        // delay to save resources
        pros::delay(25);
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intake_bottom.move_velocity(600);
            intake_top.brake();
        }

        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake_bottom.move_voltage(-12000);
            intake_top.move_voltage(-12000);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake_bottom.move_velocity(600);
            intake_top.move_velocity(600);
        }

        else{
            intake_bottom.move_voltage(0);
            intake_top.move_voltage(0);
        }
    }
    lemlib::Pose currentPose = chassis.getPose(); // Get the current pose of the chassis
        pros::lcd::print(0, "X: %f", currentPose.x);
        pros::lcd::print(1, "Y: %f", currentPose.y);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 //initialize function: chassis will be calibrated, and the center button will be registered to a callback function
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Displaying Coordinates...");
    chassis.calibrate() ; // Calibrate the chassis
   

    // Register the center button callbacks
    pros::lcd::register_btn1_cb(on_center_button);
    

// Removed duplicate definition of initialize

    // Create a task to continuously display the coordinates
    pros::Task displayCoordinates([]() {
        while (true) {
            // Print robot location to the brain screen
            //printf(0, "X: %f", chassis.getPose().x); // x-coordinate
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y-coordinate
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading (theta)

            // Delay to save resources
            pros::delay(100); // Update every 100ms
        }
    });
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
// Removed duplicate definition of autonomous

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
 /*
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}*/
// thread to for the screen and posiiion logging 
pros::Task screenTask([]() {
    while (true) {
        //print robot location to the brain screen 
        pros::lcd::print(0, "X: %f", chassis.getPose().x); //x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); //y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); //heading
        //log positiion telemetry t
        lemlib::telemetrySink() -> info("Chassis pose: {}", chassis.getPose());
        //delay to save resources
        pros::delay(50);
    
    }	

});

void autonomous() {
   // chassis.moveToPose();
    intake_bottom.move_velocity(600);   
}

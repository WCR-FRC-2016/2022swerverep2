/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveBase.h"
#include "RobotMap.h"
//#include <frc/RobotDrive.h>

DriveBase::DriveBase() : pid{frc2::PIDController(0,0,0)}, correctpid{frc2::PIDController(0,0,0)} {
	//wpi::outs() << "DriveBase constructed\n";
}

void DriveBase::DriveBaseInit() {
	//wpi::outs() << "DriveBase initialized\n";
    initialized = true;

	// Initialize all four swerve modules.
	FrontL.Initialize(frontLeftAngle, frontLeftDrive, false, robotConfig["sensorScaleFL"]);
	FrontR.Initialize(frontRightAngle, frontRightDrive, false, robotConfig["sensorScaleFR"]);
	BackL.Initialize(backLeftAngle, backLeftDrive, false, robotConfig["sensorScaleBL"]);
	BackR.Initialize(backRightAngle, backRightDrive, false, robotConfig["sensorScaleBR"]);

	// Initialize AHRS (gyro)
	ahrs = new AHRS(frc::SPI::Port::kMXP);
    //wpi::outs() << "Done setting up motor\n";

	ahrs->Reset();
	
    
	// Initialize and configure PID controller.
	pid.SetPID(robotConfig["rotateP"], robotConfig["rotateI"], robotConfig["rotateD"]);
	pid.SetSetpoint(0);
	pid.SetTolerance(2.5/180); // Tolerance of +/- 2.5 degrees.
	
    // This tells the PID controller that the input is circular; the endpoints (-1 and +1) are the same point.
    pid.EnableContinuousInput(-1.0, 1.0);
	
    
	// Initialize and configure PID controller.
	correctpid.SetPID(robotConfig["correctP"], robotConfig["correctI"], robotConfig["correctD"]);
	correctpid.SetSetpoint(0);
	correctpid.SetTolerance(.25/180); // Tolerance of +/- .25 degrees.
	
    // This tells the PID controller that the input is circular; the endpoints (-1 and +1) are the same point.
    correctpid.EnableContinuousInput(-1.0, 1.0);
}

void DriveBase::Swerve(double x, double y, double turn) {
	//wpi::outs() << "AHRS angle: " << std::to_string( ahrs -> GetAngle()) << "\n";
	
	wpi::outs().flush();

	if (x==0 && y==0 && turn==0) {
		//wpi::outs() << "Not moving.\n";
		// If not moving, don't turn the modules.
		BackR.drive(0);
		BackL.drive(0);
		FrontR.drive(0);
		FrontL.drive(0);
		return;
	}

	// Invert x and y (not sure why, but it doesn't work otherwise).
	x=-x; y=-y;

	if (robotConfig["invertTurn"]>0) {
		turn = -turn;
	}

	if (robotConfig["useTurnCorrect"]>0 && turn==0) {
		double angle = GetAngle();

		if (target_angle==-10) target_angle = angle;

		// Use the PID controller to calculate the right amount of turn.
		turn = correctpid.Calculate(target_angle, angle);
		turn = std::clamp(turn, -1.0, 1.0);

		// Don't turn the robot if it's already at the right angle.
		if (correctpid.AtSetpoint()) turn = 0;
	} else {
		target_angle = -10;
	}
	
	//wpi::outs() << "Moving. (x=" << std::to_string((double) x) << ",y=" << std::to_string((double) y) << ",turn=" << std::to_string((double) turn) << ")\n";
	
	// Calculate distance between diagonally opposite pairs of wheels.
	double r = sqrt((robotConfig["L"]*robotConfig["L"]) + (robotConfig["W"]*robotConfig["W"]));

	if (robotConfig["useFOD"]>0) {
		// Unrotate movement by current angle, except angle is backwards anyway because NavX is mounted upside down
		double theta = ahrs -> GetAngle() * PI/180; // Convert detected angle to radians
		double ox=x, oy=y; // Remember original values of x and y as we change them
		x = (ox*cos(theta)) - (oy*sin(theta));
		y = (ox*sin(theta)) + (oy*cos(theta));
	}
	
	//wpi::outs() << "Moving. (x=" << std::to_string((double) x) << ",y=" << std::to_string((double) y) << ",turn=" << std::to_string((double) turn) << ")\n";

	// Calculate x,y offsets for pairs of modules
	double a = x - turn * (robotConfig["L"] / r); // x offset for back modules
	double b = x + turn * (robotConfig["L"] / r); // x offset for front modules
	double c = y - turn * (robotConfig["W"] / r); // y offset for left modules
	double d = y + turn * (robotConfig["W"] / r); // y offset for right modules

	// Speeds (between 0 and 2*sqrt(2))
	double backRightSpeed  = sqrt((a*a)+(d*d));
	double backLeftSpeed   = sqrt((a*a)+(c*c));
	double frontRightSpeed = sqrt((b*b)+(d*d));
	double frontLeftSpeed  = sqrt((b*b)+(c*c));

	// Angles (between -1 and 1)
	double backRightAngle  = atan2(a, d) / PI;
	double backLeftAngle   = atan2(a, c) / PI;
	double frontRightAngle = atan2(b, d) / PI;
	double frontLeftAngle  = atan2(b, c) / PI;

	//wpi::outs() << "FrontL speed: " << std::to_string((double) frontLeftSpeed) << ", angle: " << std::to_string((double) frontLeftAngle) << "\n";

	BackR.drive(backRightSpeed*speed, backRightAngle);
	BackL.drive(backLeftSpeed*speed, backLeftAngle);
	FrontR.drive(frontRightSpeed*speed, frontRightAngle);
	FrontL.drive(frontLeftSpeed*speed, frontLeftAngle);
}

void DriveBase::SwerveToAngle(double x, double y, double angle) {
	// Get current angle from gyro.
	double current_angle = GetAngle();

	//wpi::outs() << "Current angle: " << std::to_string((double) current_angle) << "\n";
	//wpi::outs() << "Goal angle: " << std::to_string((double) angle) << "\n";

    // Use the PID controller to calculate the right amount of turn.
    double turn = pid.Calculate(current_angle, angle);
    turn = std::clamp(turn, -1.0, 1.0);

	//wpi::outs() << "Output: " << std::to_string((double) turn) << "\n";

    // Don't turn the robot if it's already at the right angle.
	if (pid.AtSetpoint()) {
		Swerve(x,y,0.0);
	} else {
		Swerve(x,y,turn);
	}
}

bool DriveBase::AtAngle() {
	return pid.AtSetpoint();
}

// Rotate all modules to given angles.
void DriveBase::RotateTo(double FR, double FL, double BR, double BL, bool allow_invert) {
	FrontR.drive(0, FR, allow_invert);
	FrontL.drive(0, FL, allow_invert);
	BackR.drive(0, BR, allow_invert);
	BackL.drive(0, BL, allow_invert);
}

// Run all drive motors at given speeds.
void DriveBase::SetDriveMotors(double FR, double FL, double BR, double BL) {
	FrontR.drive(FR);
	FrontL.drive(FL);
	BackR.drive(BR);
	BackL.drive(BL);
}

// Run all angle motors at given speeds.
void DriveBase::SetAngleMotors(double FR, double FL, double BR, double BL) {
	FrontR.turn(FR);
	FrontL.turn(FL);
	BackR.turn(BR);
	BackL.turn(BL);
}

void DriveBase::Periodic() {
	if (!initialized) {
		DriveBase::DriveBaseInit();
		//wpi::outs() << "Debug Statement 7\n";
	}
	//wpi::outs() << "Debug Statement 6\n";
	
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void DriveBase::Reset() {
	if (initialized) {
		// Reset gyro.
		ahrs->Reset();

		// Reset modules (and update sensor scales).
		FrontL.Reset(robotConfig["sensorScaleFL"]);
		FrontR.Reset(robotConfig["sensorScaleFR"]);
		BackL.Reset(robotConfig["sensorScaleBL"]);
		BackR.Reset(robotConfig["sensorScaleBR"]);

		// Reset PID (and update parameters)
		pid.SetPID(robotConfig["rotateP"], robotConfig["rotateI"], robotConfig["rotateD"]);
		pid.Reset();
		correctpid.SetPID(robotConfig["correctP"], robotConfig["correctI"], robotConfig["correctD"]);
		correctpid.Reset();
	}
}

// Reset just the gyro.
void DriveBase::ResetGyro() {
	if (initialized) {
		ahrs->Reset();
	}
}

// Reset all modules' angle encoders
void DriveBase::ResetEncoders() {
	if (initialized) {
		FrontL.ResetEncoder();
		FrontR.ResetEncoder();
		BackL.ResetEncoder();
		BackR.ResetEncoder();
	}
}

double DriveBase::getSpeed() {
	return speed;
}

void DriveBase::setSpeed(double newSpeed) {
	speed = std::clamp(newSpeed, 0.0, 1.0);
}

// Get current normalized angle.
double DriveBase::GetAngle() {
	double current_angle = ahrs -> GetAngle() / 180.0;
    while (current_angle<-1.0) current_angle+=2.0;
    while (current_angle>1.0) current_angle-=2.0;
	
	if (robotConfig["invertTurn"]>0) current_angle*=-1;

	return current_angle;
}

void DriveBase::DebugRawAngles() {
	wpi::outs() << "FrontL: " << std::to_string((double) FrontL.GetRawAngle()) << "\n";
	wpi::outs() << "FrontR: " << std::to_string((double) FrontR.GetRawAngle()) << "\n";
	wpi::outs() << "BackL: " << std::to_string((double) BackL.GetRawAngle()) << "\n";
	wpi::outs() << "BackR: " << std::to_string((double) BackR.GetRawAngle()) << "\n";

	wpi::outs().flush();
}
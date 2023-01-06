/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc2/command/SubsystemBase.h>
//#include <frc/controller/PIDController.h> // Included in SwerveModule.h

#include <SwerveModule.h>

#include <fstream>
#include <string>
#include <AHRS.h>

class DriveBase : public frc2::SubsystemBase {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  bool initialized = false;
  double speed = 1;

  double target_angle = -10; // -10 means currently changing. For turn-correct.

  SwerveModule FrontL;
  SwerveModule FrontR;
  SwerveModule BackL;
  SwerveModule BackR;

  AHRS * ahrs; // NavX, for gyro.

  frc2::PIDController pid;
  frc2::PIDController correctpid;

 public:
  DriveBase();
  void DriveBaseInit();
  void Periodic();
  void Reset();
  void ResetGyro();
  void ResetEncoders();
  void Swerve(double x, double y, double turn);
  void SwerveToAngle(double x, double y, double angle);
  bool AtAngle();
  void RotateTo(double FR, double FL, double BR, double BL, bool allow_invert = true);
  void SetDriveMotors(double FR, double FL, double BR, double BL);
  void SetAngleMotors(double FR, double FL, double BR, double BL);
  double getSpeed();
  void setSpeed(double newSpeed);
  double GetAngle();
  void SetAngle(double angle);
  void DebugRawAngles();
};

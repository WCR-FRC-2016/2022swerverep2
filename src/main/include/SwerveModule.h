/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "RobotMap.h"

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>

class SwerveModule {
    private:
        WPI_TalonSRX* angleMotor; // Motor to rotate the module
        WPI_TalonSRX* driveMotor; // Motor to move the robot

        frc2::PIDController pid;

        bool m_debug; // Print debug information to the console?
        double m_sensorScale;// All drive motor outputs for this module are scaled by this value.
        
    public:
        SwerveModule();
        void Initialize(int angleID, int driveID, bool debug, double sensorScale);
        void drive(double speed, double angle, bool allow_invert = true);
        void drive(double speed);
        void turn(double speed);
        void Reset(double sensorScale);
        void ResetEncoder();
        double GetRawAngle();
};
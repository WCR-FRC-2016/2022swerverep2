/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc2/command/CommandScheduler.h>
#include <cscore_oo.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  if (robotConfig["useCamera"]>0) {
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // frc::CameraServer::

    //camera.SetVideoMode(cs::VideoMode::kYUYV, 320, 240, 15);

    //wpi::outs() << camera.GetPath() << "\n";
  }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  //wpi::outs() << "Debug Statement 3\n";
  frc2::CommandScheduler::GetInstance().Run();
  //wpi::outs() << "Debug Statement 5\n";
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  // Reset the autonomous to start at the first command.
  m_container.command_no = 0;
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // Update the robotConfig table and the autonomous commands in case the files changed while the robot was disabled.
  m_container.ReadFile();
  m_container.SetConfig();

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autonomousCommand->IsFinished())
  {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
      m_autonomousCommand->Schedule();
    }
  }
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  //if (m_autonomousCommand != nullptr) {
  //  m_autonomousCommand->Cancel();
  //  m_autonomousCommand = nullptr;
  //}

  m_container.SetConfig(); // Update the robotConfig table in case the file changed while the robot was disabled.
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  m_container.UpdateDashboard();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

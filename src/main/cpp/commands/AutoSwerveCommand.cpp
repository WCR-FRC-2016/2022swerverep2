/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoSwerveCommand.h"
#include "RobotMap.h"

// Drive (left/right turn)
AutoSwerveCommand::AutoSwerveCommand(DriveBase* drivebase, double x, double y, double turn, double time) : m_drivebase{drivebase}, m_x{x}, m_y{y}, m_turn{turn}, m_time{time}, m_useAngle{false} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivebase});
}

// Drive (turn-to-angle)
AutoSwerveCommand::AutoSwerveCommand(DriveBase* drivebase, double x, double y, double angle) : m_drivebase{drivebase}, m_x{x}, m_y{y}, m_turn{angle}, m_time{-1}, m_useAngle{true} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivebase});
}

// Called when the command is initially scheduled.
void AutoSwerveCommand::Initialize() {
  m_elapsed = 0; // Reset elapsed time to 0
}

// Called repeatedly when this Command is scheduled to run
void AutoSwerveCommand::Execute() {
  // Drive
  if (m_useAngle) {
    m_drivebase->SwerveToAngle(m_x, m_y, m_turn);
  } else {
    m_drivebase->Swerve(m_x, m_y, m_turn);
  }

  //wpi::outs() << "m_time: " << std::to_string((double) m_time) << " m_elapsed: " << std::to_string((double) m_elapsed) << "\n";

  // Add 20ms (default loop update frequency) to elapsed time
  m_elapsed += 20;
}

// Called once the command ends or is interrupted.
void AutoSwerveCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoSwerveCommand::IsFinished() {
  if (m_useAngle) {
    return m_drivebase->AtAngle(); // If turn-to-angle, finished when at angle
  } else {
    return m_time>0 && m_elapsed>m_time; // If left/right turn, finished when specified time has elapsed.
  }
}

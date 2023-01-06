/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoTurnCommand.h"
#include "RobotMap.h"

AutoTurnCommand::AutoTurnCommand(DriveBase* drivebase, double FR, double FL, double BR, double BL, double time) : m_drivebase{drivebase}, m_FR{FR}, m_FL{FL}, m_BR{BR}, m_BL{BL}, m_time{time} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivebase});
}

// Called when the command is initially scheduled.
void AutoTurnCommand::Initialize() {
  m_elapsed = 0; // Reset elapsed time to 0
}

// Called repeatedly when this Command is scheduled to run
void AutoTurnCommand::Execute() {
  // Turn
  m_drivebase->SetAngleMotors(m_FR, m_FL, m_BR, m_BL);

  // Add 20ms (default loop update frequency) to elapsed time
  m_elapsed += 20;
}

// Called once the command ends or is interrupted.
void AutoTurnCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoTurnCommand::IsFinished() {
  return m_time>0 && m_elapsed>m_time;
}

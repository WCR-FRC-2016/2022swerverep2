/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveBase.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoSwerveCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoSwerveCommand> {
 public:
  AutoSwerveCommand(DriveBase* drivebase, double x, double y, double turn, double time);
  AutoSwerveCommand(DriveBase* drivebase, double x, double y, double angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  DriveBase* m_drivebase;
  double m_x;
  double m_y;
  double m_turn;
  double m_time;
  double m_elapsed;
  bool m_useAngle;
};

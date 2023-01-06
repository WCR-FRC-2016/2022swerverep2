/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//
// 2020Mar17 Walko - Added some comment and reordered some statements for better clarity
//
#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <dirent.h>

#include "subsystems/DriveBase.h"
#include "commands/AutoSwerveCommand.h"
#include "commands/AutoTurnCommand.h"
#include "RobotMap.h"
#include "frc/XboxController.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Button.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void ReadFile();
  void SetConfig();
  void UpdateDashboard();
  
  int command_no;

  bool is_calibration_mode;
  int calib_id;
  double angles[4];

  // Driver A: Toggle FOD
  frc2::Button m_driverA{[&] {return m_driverStick.GetAButton() && !is_calibration_mode;}};
  frc2::InstantCommand m_ToggleFOD{[this] {
    if (robotConfig["useFOD"]>0) robotConfig["useFOD"]=0; else robotConfig["useFOD"]=1;
  } , {&m_driveBase} };

  // Driver B: Toggle Turn Correct
  frc2::Button m_driverB{[&] {return m_driverStick.GetBButton() && !is_calibration_mode;}};
  frc2::InstantCommand m_ToggleTurnCorrect{[this] {
    if (robotConfig["useTurnCorrect"]>0) robotConfig["useTurnCorrect"]=0; else robotConfig["useTurnCorrect"]=1;
  } , {&m_driveBase} };

  // Driver X: X-mode (for defense)
  frc2::Button m_driverX{[&] {return m_driverStick.GetXButton() && !is_calibration_mode;}};
  frc2::InstantCommand m_Xmode{[this] {
    /*

    \  / 
    
    /  \ 

    */

    m_driveBase.RotateTo(-0.25, 0.25, 0.25, -0.25);
  } , {&m_driveBase} };

  // Driver Y: Reset Drivebase Gyro
  frc2::Button m_driverY{[&] {return m_driverStick.GetYButton() && !is_calibration_mode;}};
  frc2::InstantCommand m_resetGyro{[this] {m_driveBase.ResetGyro();}, {&m_driveBase}};
  
  // Driver DPad: Turn wheels without moving (for tests/calib mostly, plus helps at end of use to realign wheels)
  frc2::Button m_driverDPad{[&] {return m_driverStick.GetPOV()!=-1;}};
  frc2::InstantCommand m_rotate{[this] {
    double pass_r = m_driverStick.GetPOV();
    pass_r /= 180.0;
    if (pass_r>=1) pass_r-=2;
    if (is_calibration_mode) {
      angles[0]=pass_r; angles[1]=pass_r; angles[2]=pass_r; angles[3]=pass_r;
    } else {
      m_driveBase.RotateTo(pass_r,pass_r,pass_r,pass_r,false);
    }
  } , {&m_driveBase} };
  
  // Driver Left Trigger: Adjust speed -0.1
  frc2::Button m_driverLT{[&] {return 0.5 < m_driverStick.GetLeftTriggerAxis() && !is_calibration_mode;}};
  frc2::InstantCommand m_AdjustSpeedDown{[this] {m_driveBase.setSpeed(m_driveBase.getSpeed()-0.1);} , {&m_driveBase} };
  
  // Driver Right Trigger: Adjust speed +0.1
  frc2::Button m_driverRT{[&] {return 0.5 < m_driverStick.GetRightTriggerAxis() && !is_calibration_mode;}};
  frc2::InstantCommand m_AdjustSpeedUp{[this] {m_driveBase.setSpeed(m_driveBase.getSpeed()+0.1);} , {&m_driveBase} };
  
  // Driver Right Bumper: Swap speed to 1 or 0.5 (0.5 if at 1, otherwise 1)
  frc2::Button m_driverRB{[&] {return m_driverStick.GetRightBumper() && !is_calibration_mode;}};
  frc2::InstantCommand m_SwapSpeed{[this] {m_driveBase.setSpeed(m_driveBase.getSpeed()>0.95?0.5:1.0);} , {&m_driveBase} };

  // Driver Left Bumper: Use turn-to-angle (wrapped into drivebase default command)

  // Autonomous: Reset Gyro Angle
  frc2::InstantCommand m_ResetAngle{[this] {m_driveBase.ResetGyro();} , {&m_driveBase} };

  // Autonomous: Wait (for after EOF)
  frc2::RunCommand m_Wait{[this] {/*nothing*/} , {} };



  // Driver Select+Start: Toggle calibration mode
  frc2::Button m_driverSelectStart{[&] {return m_driverStick.GetBackButton() && m_driverStick.GetStartButton();}};
  frc2::InstantCommand m_ToggleCalib{[this] {
    is_calibration_mode = !is_calibration_mode;
    angles[0] = 0; angles[1] = 0; angles[2] = 0; angles[3] = 0;
    if (!is_calibration_mode) {
      // End calib
      m_driveBase.ResetGyro();
      if (calib_id!=4) {
        // Only reset encoders if not recently turning
        // When turning, modules are in /\ shape (not straightened)
        m_driveBase.ResetEncoders(); // \/
      }
    }
    calib_id = 0;
  } , {&m_driveBase} };

  // Driver A Calib: Next module
  frc2::Button m_driverACal{[&] {return m_driverStick.GetAButton() && is_calibration_mode;}};
  frc2::InstantCommand m_IncCalibId{[this] {
    if (calib_id==4) {
      // Change from gyro calib.
      m_driveBase.ResetGyro();
    }
    calib_id++;
    if (calib_id>=5) calib_id=0;
    if (calib_id==4) {
      // Change to gyro calib.
      m_driveBase.ResetEncoders();
    }
  } , {&m_driveBase} };

  // Driver B Calib: Previous module
  frc2::Button m_driverBCal{[&] {return m_driverStick.GetBButton() && is_calibration_mode;}};
  frc2::InstantCommand m_DecCalibId{[this] {
    if (calib_id==4) {
      // Change from gyro calib.
      m_driveBase.ResetGyro();
    }
    calib_id--;
    if (calib_id<0) calib_id=4;
    if (calib_id==4) {
      // Change to gyro calib.
      m_driveBase.ResetEncoders();
    }
  } , {&m_driveBase} };
 
 private:
  frc::XboxController m_driverStick{0};
  //frc::XboxController m_manStick{1};
  
  DriveBase m_driveBase;

  void ConfigureButtonBindings();

  std::vector<std::string> commands;
  std::fstream autofile;
  std::fstream configfile {"/home/lvuser/wcrj/config.txt"};

  // The chooser for the autonomous routines
  // ...does this do anything?
  frc::SendableChooser<std::string> m_chooser;
};
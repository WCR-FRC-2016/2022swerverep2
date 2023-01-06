/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

/* Directions:

       0
       |
 -0.5 -+- +0.5
       |
     -1=+1
*/

// Any settings that can be changed in the config file.
// See definition in RobotContainer.cpp for info on individual settings.
std::map<std::string, double> robotConfig = {
    // PID parameters for modules.
    {"angleP", 2},
    {"angleI", 0},
    {"angleD", 0},

    {"useCamera", 0}, // Enable camera? (req. code restart)
    {"useFOD", 0}, // Enable field-oriented drive?
    {"invertTurn", 0}, // Invert turns left/right? (for swapping front/back)
    {"useTurnCorrect", 0}, // Correct turning while trying to move straight?
    {"debugAngles", 0}, // Debug raw encoder angles?
    
    // PID parameters for whole robot rotation (with useAngle).
    {"rotateP", 2},
    {"rotateI", 0},
    {"rotateD", 0},
    
    // PID parameters for turn correction (with useTurnCorrect).
    {"correctP", 5},
    {"correctI", 0},
    {"correctD", 0},

    // Width and length of robot, from wheel to wheel.
    {"W", 20.35},
    {"L", 21},

    // Sensor scale for each module.
    {"sensorScaleFL", 20.0/7120.0},
    {"sensorScaleFR", 20.0/7120.0},
    {"sensorScaleBL", 20.0/16600.0},
    {"sensorScaleBR", 20.0/16600.0},

    // Drive scale for all modules (not maximum)
    {"driveScale", 0.3},

    // Threshold to invert above. Set to >1 to disable.
    {"invertThreshold", 0.75},

    // Joystick deadzones.
    {"moveDeadzone", 0.1},
    {"turnDeadzone", 0.2}
};

RobotContainer::RobotContainer() {
   SetConfig(); // Read robotConfig configs from file.

   is_calibration_mode = false;
   calib_id = 0;
   angles[0] = 0; angles[1] = 0; angles[2] = 0; angles[3] = 0;
   
      // Set up driver joystick controls (turn-to-angle)
   m_driveBase.SetDefaultCommand(frc2::RunCommand([this] {
      if (is_calibration_mode) {
         double pass_x = m_driverStick.GetLeftX();

         if (std::abs(pass_x)<=robotConfig["moveDeadzone"]) {
            pass_x = 0;
         }
         
         if (calib_id<4) {
            angles[calib_id] += pass_x/100.0;
            while (angles[calib_id]<-1.0) angles[calib_id]+=2.0;
            while (angles[calib_id]> 1.0) angles[calib_id]-=2.0;
            
            m_driveBase.RotateTo(angles[0],angles[1],angles[2],angles[3],false);
         } else {
            m_driveBase.Swerve(0,0,pass_x/2.5);
         }
      } else if (m_driverStick.GetLeftBumper()) {
         // Driver joystick controls (turn-to-angle)

         double pass_x = m_driverStick.GetLeftX();
         double pass_y = m_driverStick.GetLeftY();
         double angle_x = m_driverStick.GetRightX();
         double angle_y = m_driverStick.GetRightY();

         if (std::abs(pass_x)<=robotConfig["moveDeadzone"] && std::abs(pass_y)<=robotConfig["moveDeadzone"]) {
            pass_x=0; pass_y=0;
         }
         
         if (std::abs(angle_x)<=robotConfig["turnDeadzone"] && std::abs(angle_y)<=robotConfig["turnDeadzone"]) {
            m_driveBase.Swerve(pass_x, pass_y, 0);
         } else {
            m_driveBase.SwerveToAngle(pass_x, pass_y, atan2(-angle_x,-angle_y)/PI);
         }
      } else {
         // Driver joystick controls (left/right turn)
         double pass_x = m_driverStick.GetLeftX();
         double pass_y = m_driverStick.GetLeftY();
         double pass_turn = m_driverStick.GetRightX();

         if (std::abs(pass_x)<=robotConfig["moveDeadzone"] && std::abs(pass_y)<=robotConfig["moveDeadzone"]) {
            pass_x=0; pass_y=0;
         }
         pass_turn = (std::abs(pass_turn)>robotConfig["turnDeadzone"])?pass_turn:0;

         m_driveBase.Swerve(pass_x, pass_y, pass_turn);
      }
      if (robotConfig["debugAngles"]>0) {
         m_driveBase.DebugRawAngles();
      }
   }, {&m_driveBase}));

   // Set up autonomous command chooser
   std::vector<std::string> autoVector {};

   // Well, you know the old formula...
   DIR *dir;
   struct dirent *ent;
   if ((dir = opendir ("/home/lvuser/wcrj")) != NULL) {
      /* print all the files and directories within directory */
      while ((ent = readdir (dir)) != NULL) {
         std::string name = ent->d_name;
         // Exclude . .. config.txt
         if (name.length()>2 && name!="config.txt") autoVector.push_back(name);
      }
      closedir (dir);
   } else {
      /* could not open directory */
      perror ("");
   }

   frc::SmartDashboard::PutStringArray("Auto List", autoVector);
   
   // Set up controls.
   ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
   // Configure your button bindings here
   // See RobotContainer.h for explanations.

   m_driverA.WhenPressed(m_ToggleFOD);
   m_driverB.WhenPressed(m_ToggleTurnCorrect);
   m_driverX.WhileHeld(m_Xmode);
   m_driverY.WhileHeld(m_resetGyro);
   m_driverDPad.WhileHeld(m_rotate);
	m_driverLT.WhenPressed(m_AdjustSpeedDown);
	m_driverRT.WhenPressed(m_AdjustSpeedUp);
	m_driverRB.WhenPressed(m_SwapSpeed);

   m_driverSelectStart.WhenPressed(m_ToggleCalib);
   m_driverACal.WhenPressed(m_IncCalibId);
   m_driverBCal.WhenPressed(m_DecCalibId);
}

// Updates data on dashboard
void RobotContainer::UpdateDashboard() {
   frc::SmartDashboard::PutBoolean("DB/LED 0", robotConfig["useFOD"]>0);
   frc::SmartDashboard::PutBoolean("DB/LED 1", robotConfig["useTurnCorrect"]>0);
}

// Reads chosen autonomous file.
void RobotContainer::ReadFile() {
   // Reset file to start.
   autofile.close();
   autofile.open("/home/lvuser/wcrj/" + frc::SmartDashboard::GetString("Auto Selector", "autonomous.txt"));

   commands.clear(); // Start with empty list of commands

   // Read the file.
   std::string line;
   while (getline(autofile, line)) {
      commands.push_back(line);
   }
}

// Reads config file.
void RobotContainer::SetConfig() {
   wpi::outs() << "Reading file!\n";

   // Reset file to start.
   configfile.close();
   configfile.open("/home/lvuser/wcrj/config.txt");

   std::string line;
   while (getline(configfile, line)) {
      std::string name;
      double value;
      std::istringstream words (line);
      words >> name;
      words >> value;
      //wpi::outs() << name << " " << value << "\n";
      
      // Write to variable
      robotConfig[name] = value;
   }

   m_driveBase.Reset(); // Reset drivebase so config changes take effect.
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
   if (command_no>=(int) commands.size()) {
      return &m_Wait;
   }

   // Read next autonomous command from vector.
   std::string command = commands.at(command_no);
   command_no++;
   wpi::outs() << command << "\n";

   // Read name, arguments from command.
   std::string verb;
   std::vector<double> args;
   std::istringstream words (command);
   words >> verb;
   double num;
   while (words >> num) {
      args.push_back(num);
   }

   switch (verb[0]) {
      case 't': // Turn
         return new AutoTurnCommand(&m_driveBase, args[0], args[1], args[2], args[3], args[4]);
         break;
      case 'r': // Reset angle
         return &m_ResetAngle;
         break;
      case 'a': // Drive (turn-to-angle)
         return new AutoSwerveCommand(&m_driveBase, args[0], args[1], args[2]);
         break;
      case 'd': // Drive (left/right turn)
      default:
         return new AutoSwerveCommand(&m_driveBase, args[0], args[1], args[2], args[3]);
         break;
   }
}
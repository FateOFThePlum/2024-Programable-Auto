// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "Constants.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drive.ArcadeDrive(-m_driverController.GetLeftY(),
                            -m_driverController.GetRightX());
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton{&m_driverController, 6}
      .OnTrue(&m_driveHalfSpeed)
      .OnFalse(&m_driveFullSpeed);
}


using namespace pathplanner;

frc2::CommandPtr RobotContainer::GetAutonomousCommand(){
    return PathPlannerAuto("S Auto").ToPtr(); //Add Chooser at some point?
}

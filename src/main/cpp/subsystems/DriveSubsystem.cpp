// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
    : m_left1{kLeftMotor1Port},
      m_left2{kLeftMotor2Port},
      m_right1{kRightMotor1Port},
      m_right2{kRightMotor2Port},
      m_leftEncoder{kLeftEncoderPorts[0], kLeftEncoderPorts[1]},
      m_rightEncoder{kRightEncoderPorts[0], kRightEncoderPorts[1]},
      m_odometry{frc::Rotation2d(m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kY)), units::meter_t{0}, units::meter_t{0}} {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);

  m_left1.AddFollower(m_left2);
  m_right1.AddFollower(m_right2);

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_right1.SetInverted(true);

  // Set the distance per pulse for the encoders
  m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());
  m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse.value());

  ResetEncoders();
  
  //Configure AutoBuilder For PathPlanner
  AutoBuilder::configureRamsete(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetChassisSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ ChassisSpeedTankDrive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        ReplanningConfig(), // Default path replanning config. See the API for the options here
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  frc::SmartDashboard::PutNumber("Left", GetLeftEncoder().GetDistance());
  frc::SmartDashboard::PutNumber("Right", GetRightEncoder().GetDistance());
  m_odometry.Update(frc::Rotation2d(m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kY)),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::ChassisSpeedTankDrive(frc::ChassisSpeeds ChassisSpeeds){
  frc::DifferentialDriveKinematics kinematics{21.75_in}; //Correct for 2023 Robot
  frc::SimpleMotorFeedforward<units::meters> left_feedforward(left_kS, left_kV, left_kA); //Put this in a diffent file so it is only called once "exampleClass::feedfoward"?
  frc::SimpleMotorFeedforward<units::meters> right_feedforward(right_kS, right_kV, right_kA); 

  auto [left, right] = kinematics.ToWheelSpeeds(ChassisSpeeds);
  
  m_left1.SetVoltage(left_feedforward.Calculate(left));
  m_left2.SetVoltage(left_feedforward.Calculate(left));
  m_right1.SetVoltage(right_feedforward.Calculate(right));
  m_right1.SetVoltage(right_feedforward.Calculate(right));
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_left1.SetVoltage(left);
  m_right1.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

frc::Encoder& DriveSubsystem::GetLeftEncoder() {
  return m_leftEncoder;
}

frc::Encoder& DriveSubsystem::GetRightEncoder() {
  return m_rightEncoder;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kY);
}

//double DriveSubsystem::GetTurnRate() {
//  return -m_gyro.GetRate();
//}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t{m_leftEncoder.GetRate()},
          units::meters_per_second_t{m_rightEncoder.GetRate()}};
}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds(){ //Get Wheel speeds from GetWheelSpeeds and returns Chassis Speeds
  frc::DifferentialDriveKinematics kinematics{21.75_in}; //Actual Center distance Between Wheels is 21.75 in or 0.55245 meter
  return kinematics.ToChassisSpeeds(GetWheelSpeeds());
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(frc::Rotation2d(m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kY)),
                           units::meter_t{m_leftEncoder.GetDistance()},
                           units::meter_t{m_rightEncoder.GetDistance()}, pose);
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int driveCoderID, int turningCoderID);
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kEncoderResolution = 4096;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 2_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 3_rad_per_s / 1_s;  // radians per second^2

  ctre::phoenix::motorcontrol::can::TalonFX m_driveMotor;
  ctre::phoenix::motorcontrol::can::TalonFX m_turningMotor;

  ctre::phoenix::sensors::CANCoder m_driveEncoder;
  ctre::phoenix::sensors::CANCoder m_turningEncoder;

  frc::PIDController m_drivePIDController{0.1, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      0.1,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
                                                                3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
      1_V, 0.5_V / 1_rad_per_s};
};

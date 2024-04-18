// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int driveCoderID,
                           const int turningCoderID)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_driveEncoder(driveCoderID),
      m_turningEncoder(turningCoderID) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.ConfigSensorDirection(false); // Set sensor direction if needed
  m_driveEncoder.ConfigFeedbackCoefficient(2 * std::numbers::pi / kEncoderResolution, "rad", SensorTimeBase::PerSecond); // Set feedback coefficient for radians
  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.

  m_turningEncoder.ConfigFeedbackCoefficient(2 * std::numbers::pi / kEncoderResolution, "rad", SensorTimeBase::PerSecond); // Set feedback coefficient for radians

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{const_cast<ctre::phoenix::sensors::CANCoder&>(m_driveEncoder).GetVelocity()},
          units::radian_t{const_cast<ctre::phoenix::sensors::CANCoder&>(m_turningEncoder).GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{const_cast<ctre::phoenix::sensors::CANCoder&>(m_driveEncoder).GetPosition()},
          units::radian_t{const_cast<ctre::phoenix::sensors::CANCoder&>(m_turningEncoder).GetPosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetPosition()}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetPosition()}, state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.Set(TalonFXControlMode::PercentOutput, driveOutput + driveFeedforward.to<double>());
  m_turningMotor.Set(TalonFXControlMode::PercentOutput, turnOutput + turnFeedforward.to<double>());
}
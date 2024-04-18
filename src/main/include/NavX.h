/*
class NavX

Constructors

    NavX(const int&)
        Creates a NavX on the specified interface (kUSB, kMXP)

Public Methods

    double getYaw()
        Returns the yaw value.
    double getYawFull()
        Returns the yaw value from 0-360.
        Rolls over at extremes; used with the standard unit circle.
    double getAngle()
        Returns the angle value (-infinity to infinity, beginning at 0).
    double getAbsoluteAngle()
        Returns the absolute value of the angle value.
    void resetYaw()
        Sets the yaw value to zero.
    void resetAll()
        Resets all NavX return values and calibrates the sensor.

    enum ConnectionType
        Used with the constructor to specify which interface to construct on.
*/

#pragma once

#include <math.h>

#include "AHRS.h"

#include <frc/Errors.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Rotation2d.h> // Add this include for Rotation2d

class NavX {

    public:
        static NavX& GetInstance() {
            static NavX* instance = new NavX(ConnectionType::kMXP);
            return *instance;
        }
        double getRoll(){
            return navX->GetPitch();
        }
        double getPitch(){
            return navX->GetRoll();
        }

        double getYaw() {

            return navX->GetYaw();
        }
        double getYawFull(){

            if (getYaw() < 0) {

                return getYaw() + 360;
            }
            else {

                return getYaw();
            }
        }
        double getAngle() {

            return navX->GetAngle();
        }
        double getAbsoluteAngle() {

            return abs(navX->GetAngle());
        }

        void resetYaw() {

            if (isCalibrating()) {
                
                FRC_ReportError(frc::warn::Warning, "Attempting to reset yaw... This might not work as the NavX is not calibrated yet! {}", -1);
            }
            else {

                
            }
            navX->ZeroYaw();
        }
        void resetAll() {

            navX->Reset();
        }

        void Calibrate() {

            navX->Calibrate();
        }

        bool isCalibrating() {
            return navX->IsCalibrating();
        }

        enum ConnectionType {

            kUSB,
            kMXP = 4
        };
        void setRobotOrientation(const frc::Pose2d& robotPose) {
            // Extract the rotation from the robot's pose
            frc::Rotation2d rotation = robotPose.Rotation();
            // Get the angle of the rotation
            double angle = rotation.Degrees().to<double>();

            // Set the robot's orientation
            NavX::GetInstance().setOrientation(angle);
        }
        void setOrientation(double angle) {
            // Zero the yaw to set the current angle as 0
            navX->ZeroYaw();
            // Calculate the adjustment to set the desired angle
            double adjustment = angle - getYaw();
            // Set the adjustment to the NavX
            navX->SetAngleAdjustment(adjustment);
        }

    private:
        NavX(const int &connectionType) {
            if (connectionType == kUSB) {
                navX = new AHRS(frc::SPI::Port::kOnboardCS0);
            } else if (connectionType == kMXP) {
                navX = new AHRS(frc::SPI::Port::kMXP);
            } else {
                navX = new AHRS(frc::SPI::Port::kMXP);
            }
        }

        NavX(const NavX&) = delete;
        NavX& operator = (const NavX&) = delete;
        NavX(NavX&&) = delete;
        NavX& operator = (NavX&&) = delete;

        AHRS *navX;

        
    
};



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2914.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDriveDeadband = 0.05;
    }

    public static final class ArmConstants {
        public static final int armMotor = 0;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        // TODO: update chassis parameters
        public static final double kTrackWidth = Units.inchesToMeters(25.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(25.5);

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = 0;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = 0;
        public static final double kBackRightChassisAngularOffset = Math.PI;

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // SPARK MAX CAN IDs
        // TODO: update CAN IDs
        public static final int kFrontLeftDrivingCanId = 4;
        public static final int kRearLeftDrivingCanId = 17;
        public static final int kFrontRightDrivingCanId = 12;
        public static final int kRearRightDrivingCanId = 14;

        public static final int kFrontLeftTurningCanId = 5;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 7;
        public static final int kRearRightTurningCanId = 9;

        public static final boolean kGyroReversed = false;
    }

    public static final class VisionConstants {

        public static boolean closeAllignable(int aprilId){
            return (aprilId == 6 || aprilId == 1 || aprilId == 5 || aprilId == 7 || aprilId == 12 || aprilId == 10 || aprilId == 22);
        }

        public static String frontCameraName = "HD_Pro_Webcam_C920";
        public static int camFPS = 5;

        public static int avrgMs = 6;
    }
    
    public static final class AutoConstants {
            public static PIDConstants PID_X_CONSTANTS = new PIDConstants(3.5   , 0, .3);
            public static PIDConstants PID_ROT_CONSTANTS = new PIDConstants(5, 0, 0);

            public static double MASS_KG = 0;
            public static double MOI = 0;

            //TODO: IF WE DONT USE UI WE NEED TO GET THESE NUMBERS IN
            //public static RobotConfig ROBOT_CONFIG = new RobotConfig(MASS_KG, MOI, new ModuleConfig(null, null, MASS_KG, null, null, 0), null);
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 15; // amps

        public static final double kGlobalRampRate = 0.02;
    }

    public static final class IntakeConstants {
        public static final int spinnerCanId = 0;
        public static final double intakeSpeed = 0.3;
    }
}

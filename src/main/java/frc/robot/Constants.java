// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kDebugMode = false;

  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);

    public static final Pose2d kOppositeField = new Pose2d(kFieldLengthMeters, kFieldWidthMeters,
        Rotation2d.fromDegrees(180));
  }

  public static final class DriveConstants {
    public static final double kWheelDiameter = Units.inchesToMeters(3.87);

    public static final double kWheelBase = Units.inchesToMeters(23);
    public static final double kTrackWidth = Units.inchesToMeters(23);

    public static final Translation2d[] kSwerveModuleTranslations = {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0), //values for front left (+, +)
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0), //values for front right (+, -)
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0), //values for back left (-, +)
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0) //values for back right (-, -)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kSwerveModuleTranslations);

    // Gear Ratios
    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 18;

    // Encoder Conversion Factors
    public static final double kDrivePositionConversionFactor = Math.PI * kWheelDiameter / kDriveGearRatio;
    public static final double kTurnPositionConversionFactor = 2 * Math.PI / kTurnGearRatio;

    // PID Values
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.001;
    public static final double kDriveFF = 2.96;

    public static final double kTurnP = 0.1;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.005;

    // Max Speeds
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxSpeedRadiansPerSecond = Units.degreesToRadians(360);
  }

  public static final class ElectricalConstants {
    // Front Left Swerve Module Channels
    public static final int kFrontLeftTurnMotorPort = 8;
    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kFrontLeftCANCoderPort = 3;

    // Front Right Swerve Module Channels
    public static final int kFrontRightTurnMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kFrontRightCANCoderPort = 2;

    // Back Left Swerve Module Channels
    public static final int kBackLeftTurnMotorPort = 2;
    public static final int kBackLeftDriveMotorPort = 1;
    public static final int kBackLeftCANCoderPort = 0;

    // Back Right Swerve Module Channels
    public static final int kBackRightTurnMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 3;
    public static final int kBackRightCANCoderPort = 1;

    // Gyro Channel
    public static final int kGyroPort = 0;

    // Elevator Spark Max Channels
    public static final int kElevatorLeaderPort = 12;
    public static final int kElevatorFollowerPort = 13;
  }

  public static final class VisionConstants {
    public static final String kPhotonVisionIp = "10.4.22.22";
    public static final String kPhotonVisionHostName = "photonvision.local";
    public static final int kPhotonVisionPort = 5800;

    public static final double kAprilTagCameraFOVDiag = 110.0;
    public static final String kAprilTagCameraName = "limelight";
    public static final Transform3d kAprilTagRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(15.5),
            Units.inchesToMeters(-0.6),
            Units.inchesToMeters(10.8375)),
        new Rotation3d());
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.5;
    public static final double kMaxAccelMetersPerSecondSq = 1.5;
  }
}

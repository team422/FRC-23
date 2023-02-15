// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);
  }

  public static final class DriveConstants {
    public static final double kWheelDiameter = Units.inchesToMeters(3.0);
    public static final double kWheelBase = Units.inchesToMeters(23);
    public static final double kTrackWidth = Units.inchesToMeters(23);

    public static final Translation2d[] kSwerveModuleTranslations = {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0), //values for front left (+, +)
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0), //values for front right (+, -)
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0), //values for back left (-, +)
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0) //values for back right (-, -)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kSwerveModuleTranslations);
  }

  public static final class ElectricalConstants {
    // LED constants
    public static final int kLEDLength = 84;
    public static final int kLEDPort = 9;

    // Front Left Swerve Module Channels
    public static final int kFrontLeftTurnMotorPort = 16;
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftCANCoderPort = 7;

    // Front Right Swerve Module Channels
    public static final int kFrontRightTurnMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 23;
    public static final int kFrontRightCANCoderPort = 1;

    // Back Left Swerve Module Channels
    public static final int kBackLeftTurnMotorPort = 32;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kBackLeftCANCoderPort = 2;

    // Back Right Swerve Module Channels
    public static final int kBackRightTurnMotorPort = 39;
    public static final int kBackRightDriveMotorPort = 38;
    public static final int kBackRightCANCoderPort = 3;

    // Gyro Channel
    public static final int kGyroPort = 22;
  }

  public static final class VisionConstants {

  }
}

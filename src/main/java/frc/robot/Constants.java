// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final boolean tuningMode = true;
  public static final MechTechRobots CURRENT_ROBOT = MechTechRobots.MARK2;

  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);
  }

  public static final class DriveConstants {
    public static final double kWheelDiameter = Units.inchesToMeters(3.7);

    public static final double kWheelBase = Units.inchesToMeters(23);
    public static final double kTrackWidth = Units.inchesToMeters(23);
    public static Translation2d[] kModuleTranslations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
    };
    public static final Pose2d startPose = new Pose2d(3, 5, new Rotation2d());
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
  }

  public static final class Ports {
    public static final int pigeonPort = 0;
  }
}

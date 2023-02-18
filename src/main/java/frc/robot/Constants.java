// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

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

  public static final class ElevatorConstants {
    public static final ProfiledPIDController elevatorPIDController = new ProfiledPIDController(0.1, 0, 0,
        new Constraints(.5, .2));
    public static final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(0.1, 0.1, 0.1);
    public static final double elevatorGearRatio = 1.0;
    public static final int elevatorEncoderCPR = 4096;
  }

  public static final class IntakeConstants {
    public static final PIDController intakePIDController = new PIDController(0.1, 0, 0);
    public static final double intakeGearRatio = 1.0;
  }

  public static final class DriveConstants {

    public static final double kDriveDeadband = 0.1;

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
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 0.005 * Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 0.000005 * Math.PI;
  }

  public static final class ModuleConstants {
    public static final TunableNumber kDriveP = new TunableNumber("Drive P", 0.1);
    public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0);
    public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.00);
    public static final TunableNumber kDriveFF = new TunableNumber("Drive FF", 2.96);

    public static final TunableNumber kTurningP = new TunableNumber("TrP", 0.10);
    public static final TunableNumber kTurningI = new TunableNumber("Turnin I", 0.00);
    public static final TunableNumber kTurningD = new TunableNumber("Turnin D", 0.005);

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.85); // 0.09398; // 3.7 in

    public static final double kDriveGearRatio = 6.75;
    // public static final double kDriveConversionFactor = ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio);
    public static final double kDriveConversionFactor = 1 / 22.0409;

    public static final double kTurnPositionConversionFactor = 21.428;

  }

  public static final class Ports {
    public static final int pigeonPort = 23;
    public static final int intakeMotorPort = 0;
    public static final int wristMotorPort = 0;
    public static final int wristThroughborePort = 0;
    public static final int elevatorLeaderMotorPort = 0;
    public static final int elevatorFollowerMotorPort = 0;
    public static final int elevatorThroughBoreEncoderPortA = 0;
    public static final int elevatorThroughBoreEncoderPortB = 0;
  }

  public static final class VisionConstants {
    public static final int kAprilTagPipelineIndex = 0;
    public static final int kReflectiveTapePipelineIndex = 1;
    public static final String klimelightName = "limelight";
    public static final Transform3d klimelightTransform = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d());
    public static final String kfrontCameraName = "frontCamera";
    public static final Transform3d kfrontCameraTransform = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d());
    public static final String kbackCameraName = "backCamera";
    public static final Transform3d kbackCameraTransform = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d());

  }

  public static final class WristConstants {
    public static final int wristEncoderCPR = 4096; // Counts per revolution
    public static final PIDController wristPIDController = new PIDController(0.0001, 0, 0);

  }

  public static final class OIConstants {
    public static final int kDriverLeftDriveStickPort = 0;
    public static final int kDriverRightDriveStickPort = 1;

  }
}

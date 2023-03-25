// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.util.CustomHolmonomicDrive;
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

  public static final class MetaConstants {
    public static final boolean pathTuningMode = true;
  }

  public static final class RobotConstants {

    public static final double robotWidth = Units.inchesToMeters(27.5);
    public static final boolean AScopeLogging = true;
  }

  public static final class Setpoints {
    // FORMAT is ELEVATOR height in METERS and then INTAKE angle in DEGREES
    public static final double[] pickUpConeVerticalCommandSetpoints = { Units.inchesToMeters(18), -23.95 - 4.25 }; // OLD might revert
    // public static final double[] pickUpConeVerticalCommandSetpoints = { Units.inchesToMeters(20), -33.95 };

    public static final double[] pickUpCubeGroundCommandSetpoints = { Units.inchesToMeters(0), 10 - 4.25 };
    public static final double[] pickUpConeGroundCommandSetpoints = { Units.inchesToMeters(0), -19 - 4.25 };
    public static final double[] intakeFromLoadingStationCommand = { Units.inchesToMeters(8.2), 12 - 4.25 };
    public static final double[] dropLoadingStationCommandSetpoints = { Units.inchesToMeters(0), 74.4 - 4.25 };
    public static final double[] coneMidCommandSetpoints = { Units.inchesToMeters(42), -25 - 4.25 };
    public static final double[] cubeMidCommandSetpoints = { Units.inchesToMeters(35), 12 - 4.25 };
    public static final double[] cubeHighCommandSetpoints = { Units.inchesToMeters(44.489), 40 };
    public static final double[] cubeHighCommandSetpointsAuto = { Units.inchesToMeters(47), 50 };
    public static final double[] coneHighCommandSetpoints = { Units.inchesToMeters(51), -5 - 4.25 };
    public static final double[] stowVerticalCommandSetpoints = { Units.inchesToMeters(0), 95 - 4.25 };
    // side is considered the side of the field without drivers, wall has drivers
    public static final ExtendedPathPoint blueLeftWallLoadingStation = new ExtendedPathPoint(
        new Translation2d(15.8, 7.37),
        new Rotation2d(), Rotation2d.fromDegrees(0));
    public static final ExtendedPathPoint blueRightWallLoadingStation = new ExtendedPathPoint(
        new Translation2d(15.8, 6.0),
        new Rotation2d(), Rotation2d.fromDegrees(0));
    // Grid is labeled first to third from edge of field without 
    public static final ExtendedPathPoint blueFirstGridLeftCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 0.43),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint centerOfChargeStation = new ExtendedPathPoint(new Translation2d(3.88, 2.94),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueFirstGridCube = new ExtendedPathPoint(new Translation2d(1.84 + .4, 1.08),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueFirstGridRightCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 1.61),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueSecondGridLeftCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 2.16),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueSecondGridCube = new ExtendedPathPoint(new Translation2d(1.84 + .4, 2.75),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueSecondGridRightCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 3.34),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueThirdGridLeftCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 3.89),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueThirdGridCube = new ExtendedPathPoint(new Translation2d(1.84 + .4, 4.42),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueThirdGridRightCone = new ExtendedPathPoint(
        new Translation2d(1.84 + .4, 4.97),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueLeftOfBalance = new ExtendedPathPoint(new Translation2d(3.74, 4.72),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint blueRightOfBalance = new ExtendedPathPoint(new Translation2d(3.74, 0.68),
        new Rotation2d(), Rotation2d.fromDegrees(180));
    public static final ExtendedPathPoint bluePreLoadingStation = new ExtendedPathPoint(new Translation2d(12.77, 6.36),
        new Rotation2d(), Rotation2d.fromDegrees(90));
    public static final ExtendedPathPoint redLeftWallLoadingStation = blueLeftWallLoadingStation.flipPathPoint();
    public static final ExtendedPathPoint redRightWallLoadingStation = blueRightWallLoadingStation.flipPathPoint();
    public static final ExtendedPathPoint redFirstGridLeftCone = blueFirstGridLeftCone.flipPathPoint();
    public static final ExtendedPathPoint redFirstGridCube = blueFirstGridCube.flipPathPoint();
    public static final ExtendedPathPoint redFirstGridRightCone = blueFirstGridRightCone.flipPathPoint();
    public static final ExtendedPathPoint redSecondGridLeftCone = blueSecondGridLeftCone.flipPathPoint();
    public static final ExtendedPathPoint redSecondGridCube = blueSecondGridCube.flipPathPoint();
    public static final ExtendedPathPoint redSecondGridRightCone = blueSecondGridRightCone.flipPathPoint();
    public static final ExtendedPathPoint redThirdGridLeftCone = blueThirdGridLeftCone.flipPathPoint();
    public static final ExtendedPathPoint redThirdGridCube = blueThirdGridCube.flipPathPoint();
    public static final ExtendedPathPoint redThirdGridRightCone = blueThirdGridRightCone.flipPathPoint();
    public static final ExtendedPathPoint redLeftOfBalance = blueLeftOfBalance.flipPathPoint();
    public static final ExtendedPathPoint redRightOfBalance = blueRightOfBalance.flipPathPoint();
    public static final ExtendedPathPoint redPreLoadingStation = bluePreLoadingStation.flipPathPoint();

    public static final Rotation2d kIntakeApproachAngleHighCone = Rotation2d.fromDegrees(-5);
    public static final Rotation2d kIntakeApproachAngleMidCone = Rotation2d.fromDegrees(-25);
    public static final Rotation2d kIntakeApproachAngleHighCube = Rotation2d.fromDegrees(22);
    public static final Rotation2d kIntakeApproachAngleMidCube = Rotation2d.fromDegrees(12);

    public static final double distanceToDropCone = Units.inchesToMeters(4);
  }

  public static final class FieldConstants {
    public static final double kFieldLengthMeters = Units.feetToMeters(54.27083);
    public static final double kFieldWidthMeters = Units.feetToMeters(26.2916);
    public static final Pose2d kOppositeField = new Pose2d(kFieldLengthMeters, kFieldWidthMeters,
        Rotation2d.fromDegrees(180));
  }

  public static final class LEDConstants {
    // LED constants
    public static final int kLEDLength = 40;
    public static final int kLEDPort = 0;
    public static final int kLEDPort2 = 1;
  }

  public static final class ElevatorConstants {
    public static final boolean kTuningMode = true;

    public static final TunableNumber kP = new TunableNumber("Elevator P", 12.8);
    public static final TunableNumber kManualSetpoint = new TunableNumber("Elevator Height", 0.0);
    public static final TunableNumber kI = new TunableNumber("Elevator I", 1.6);
    public static final TunableNumber kD = new TunableNumber("Elevator D", 0.3);
    public static final TunableNumber kKs = new TunableNumber("Elevator ks", .1);
    public static final TunableNumber kKg = new TunableNumber("Elevator kg", .37);
    public static final TunableNumber kKv = new TunableNumber("Elevator kv", 0.1);

    public static final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(kKs.get(),
        kKg.get(), kKv.get());
    public static final ProfiledPIDController elevatorPIDController = new ProfiledPIDController(kP.get(),
        kI.get(), kD.get(),
        new Constraints(30, 6));

    public static final double kGearRatio = 2.256 * Math.PI;
    public static final int kEncoderCPR = 2048;

    public static final Rotation2d kAngle = Rotation2d.fromDegrees(56);
    public static final double kMaxTravelMeters = Units.inchesToMeters(55);

    public static final double kMinHeightMeters = Units.inchesToMeters(6.566);
    public static final double kMaxHeightMeters = kMinHeightMeters + kMaxTravelMeters * kAngle.getSin();

    public static final double kElevatorMassKG = 5;
    public static final double kDrumSize = Units.inchesToMeters(2.256);
    public static final double kCarriageArmLength = Units.inchesToMeters(16.556);

    public static final double kDistanceToCenterOfRobot = Units.inchesToMeters(12.0);
  }

  public static final class IntakeConstants {
    public static final PIDController intakePIDController = new PIDController(0.1, 0, 0);
    public static final double intakeGearRatio = 1.0;
    public static final double intakeLengthMeters = Units.inchesToMeters(17.0);
  }

  public static final class DriveConstants {

    public static final double kDriveDeadband = 0.04;

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
    public static final double kMaxModuleSpeedMetersPerSecond = 6;
    public static final double kMaxSpeedMetersPerSecond = 8.5; // 8.5
    public static final double kMaxHighElevatorSpeedMetersPerSecond = 2.5; // 8.5
    public static final double kMaxAccelMetersPerSecondSq = 4;

    public static final double kMaxAcceptedErrorMeters = 0.5;
    public static final Rotation2d kMaxAcceptedAngleError = Rotation2d.fromDegrees(10);

    public static final double kMaxSpeedMetersPerSecondAuto = 3.85;
    public static final double kMaxAccelMetersPerSecondSqAuto = 2.5;

    public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(360);
    public static final double kMaxHighElevatorAngularSpeedRadiansPerSecond = Units.degreesToRadians(120);

    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Units.degreesToRadians(180);
    public static final Rotation2d pitchAngle = Rotation2d.fromDegrees(-1.17);
    public static final CustomHolmonomicDrive holonomicDrive = new CustomHolmonomicDrive(new PIDController(1.0, 0, 0),
        new PIDController(.03, 0, 0), new SlewRateLimiter(kMaxAccelMetersPerSecondSq),
        new SlewRateLimiter(kMaxAccelMetersPerSecondSq),
        new SlewRateLimiter(kMaxAngularAccelerationRadiansPerSecondSquared));
  }

  public static final class ModuleConstants {
    public static final TunableNumber kDriveP = new TunableNumber("Drive P", 0.1);
    public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0);
    public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.00);
    public static final TunableNumber kDriveFF = new TunableNumber("Drive FF", 2.96);

    public static final TunableNumber kTurningP = new TunableNumber("TrP", 0.05);
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

    public static final int intakeMotorPort = 4;

    public static final int wristMotorPort = 5;
    public static final int wristThroughborePort = 4;

    public static final int elevatorLeaderMotorPort = 1;
    public static final int elevatorFollowerMotorPort = 2;

    public static final int elevatorThroughBoreEncoderPortA = 1;
    public static final int elevatorThroughBoreEncoderPortB = 0;

    // Left Front Ports
    public static final int leftFrontDrivingMotorPort = 12;
    public static final int leftFrontTurningMotorPort = 7;
    public static final int leftFrontCanCoderPort = 18;

    // Right Front Ports
    public static final int rightFrontDriveMotorPort = 6;
    public static final int rightFrontTurningMotorPort = 39;
    public static final int rightFrontCanCoderPort = 17;

    // Left Rear Ports
    public static final int leftRearDriveMotorPort = 9;
    public static final int leftRearTurningMotorPort = 11;
    public static final int leftRearCanCoderPort = 16;

    // Right Rear Ports
    public static final int rightRearDriveMotorPort = 3;
    public static final int rightRearTurningMotorPort = 8;
    public static final int rightRearCanCoderPort = 15;
  }

  public static final class VisionConstants {
    public static final int kAprilTagPipelineIndex = 0;
    public static final int kReflectiveTapePipelineIndex = 1;
    public static final String klimelightName = "limelight";
    public static final Transform3d klimelightTransform = new Transform3d(new Translation3d(0, 0, 0),
        new Rotation3d());
    public static final String kRightCamera = "AprilTagCameraGreen";
    public static final Transform3d kRightCameraTransform = new Pose3d(new Translation3d(
        Units.inchesToMeters(6.366), Units.inchesToMeters(-8.055), Units.inchesToMeters(36.65)),
        new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(-15))).minus(new Pose3d());
    public static final String kleftCameraName = "AprilTagCameraGray";
    public static final Transform3d kleftCameraTransform = new Pose3d(new Translation3d(
        Units.inchesToMeters(6.366), Units.inchesToMeters(8.055), Units.inchesToMeters(36.65)),
        new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15))).minus(new Pose3d());
  }

  public static final class WristConstants {
    public static final TunableNumber kWristSetpoint = new TunableNumber("Wrist degrees", 0.0);
    public static final TunableNumber kWristAccel = new TunableNumber("Wrist accel", 12.0);
    public static final TunableNumber kWristVelo = new TunableNumber("Wrist Velo", 4.0);
    public static final TunableNumber kWristP = new TunableNumber("Wrist P", 3.8);
    public static final TunableNumber kWristI = new TunableNumber("Wrist I", 0.08);
    public static final TunableNumber kWristD = new TunableNumber("Wrist D", 0.15);
    public static final TunableNumber kWristks = new TunableNumber("Wrist ks", 0.0);
    public static final TunableNumber kWristkg = new TunableNumber("Wrist kg", .53);
    public static final TunableNumber kWristkv = new TunableNumber("Wrist kv", 0.0);
    public static final TunableNumber kWristka = new TunableNumber("Wrist ka", 0.0);
    public static final boolean kWristTuning = false;

    public static final int wristEncoderCPR = 4096; // Counts per revolution

    public static final ArmFeedforward wristFeedForward = new ArmFeedforward(kWristks.get(), kWristkg.get(),
        kWristkv.get(), kWristka.get());
    public static final ProfiledPIDController wristPIDController = new ProfiledPIDController(kWristP.get(),
        kWristI.get(), kWristD.get(),
        new Constraints(kWristVelo.get(), kWristAccel.get()));

    public static final double kLengthMeters = Units.inchesToMeters(3);
    public static final double kGearRatio = 34.8444444444;
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(95);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-80);
  }

  public static final class OIConstants {
    public static final int kDriverLeftDriveStickPort = 0;
    public static final int kDriverRightDriveStickPort = 1;
  }
}

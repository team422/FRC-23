package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Boolean tuningMode = true;

  public static final class Vision {
    public static final String firstCameraName = "limelight";
  }

  public static final class ElectricalConstants {

    //LED Things
    public static final int LEDPWMPort = 9;
    public static final int LEDLength = 422;

    // public static final PneumaticsModuleType kPneumaticHub = PneumaticsModuleType.CTREPCM;
    // public static final ModuleType kPowerDistributionModule = ModuleType.kCTRE;

    public static final int kRobotControllerPort = 0;
    public static final int kPowerDistributionPort = 1;

    // public static final int kRearRightDriveMotorPort = 10;
    // public static final int kFrontRightDriveMotorPort = 12;
    // public static final int kFrontLeftDriveMotorPort = 22;
    // public static final int kRearLeftDriveMotorPort = 24;

    // public static final int kRearRightTurningMotorPort = 11;
    // public static final int kFrontRightTurningMotorPort = 13;
    // public static final int kFrontLeftTurningMotorPort = 23;
    // public static final int kRearLeftTurningMotorPort = 25;

    // public static final int kRearRightTurningEncoderPort = 31;
    // public static final int kFrontRightTurningEncoderPort = 33;
    // public static final int kFrontLefTurningEncoderPort = 43;
    // public static final int kRearLeftTurningEncoderPort = 45;

    public static final int kGyroPort = 20;

    // public static final int kIntakeRetractPort = 1; // Pneumatic Control Module Port, not CAN ID
    // public static final int kIntakeDeployPort = 0; // Pneumatic Control Module Port, not CAN ID
    // public static final int kIntakeBottomPort = 21;
    // public static final int kIntakeTopPort = 20;

    // public static final int kTriggerPort = 14;

    // public static final int kShooterHoodPort = 15;
    // public static final int kShooterLeaderPort = 16;
    // public static final int kShooterFollowerPort = 17;

    // public static final int kClimberDeployPort = 2;
    // public static final int kClimberRetractPort = 3;

  }

  public static final class DriveConstants {

    // Define the conventional order of our modules when putting them into arrays
    // public static final int FRONT_LEFT = 0;
    // public static final int FRONT_RIGHT = 1;
    // public static final int REAR_LEFT = 2;
    // public static final int REAR_RIGHT = 3;

    // public static final class CANCoder {
    //     public static final double kRearRightTurningEncoderOffset = 180.0;
    //     public static final double kFrontRightTurningEncoderOffset = 180.0;
    //     public static final double kFrontLefTurningEncoderOffset = 180.0;
    //     public static final double kRearLeftTurningEncoderOffset = 180.0;
    // }

    // public static final boolean kFrontLeftDriveEncoderReversed = false;
    // public static final boolean kFrontRightDriveEncoderReversed = false;
    // public static final boolean kRearLeftDriveEncoderReversed = true;
    // public static final boolean kRearRightDriveEncoderReversed = true;
    // public static final int kSwerveTestMotorTurning = 38;
    // public static final int analogEncoderSwerveTesting = 3;
    // public static final int kSwerveTestMotorDrive = 39;
    // public static final int kSwerveTestMotorTurning2 = 40; // SET REAL VALUES
    // public static final int kSwerveTestMotorDrive2 = 41;
    // public static final int analogEncoderSwerveTesting2 = 42;
    // public static final boolean kFrontLeftTurningEncoderReversed = false;
    // public static final boolean kFrontRightTurningEncoderReversed = false;
    // public static final boolean kRearLeftTurningEncoderReversed = true;
    // public static final boolean kRearRightTurningEncoderReversed = true;
    public static final double kMaxAngularSpeed = 5 * Math.PI; // radians per second

    // Units are meters.
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23); // 22.5 in

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23); // 24.5 in

    // Units are meters per second
    // public static final double kMaxTranslationalVelocity = 2; //max 4.5

    // Units are radians per second
    public static final double kMaxRotationalVelocity = .01 * Math.PI; //max 5.0
    public static final double kMaxSpeedMetersPerSecond = 6;
    // // Max velocity

    // //The locations for the modules must be relative to the center of the robot. 
    // // Positive x values represent moving toward the front of the robot 
    // // Positive y values represent moving toward the left of the robot.

    public static Translation2d[] kModuleTranslations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);

    public static final int kFrontRightDriveMotor = 23;
    public static final int kFrontRightTurningMotor = 2;
    public static final int kFrontRightEncoder = 1;

    public static final int kFrontLeftDriveMotor = 1;
    public static final int kFrontLeftTurningMotor = 16;
    public static final int kFrontLeftEncoder = 7;

    public static final int kRearRightDriveMotor = 38;
    public static final int kRearRightTurningMotor = 39;
    public static final int kRearRightEncoder = 3;

    public static final int kRearLeftDriveMotor = 3;
    public static final int kRearLeftTurningMotor = 32;
    public static final int kRearLeftEncoder = 2;
    public static final int kGyroPort = 22;
    public static final double kMaxAutoSpeedMetersPerSecond = 2.0;
    public static final double KMaxAutoRotationPerSecond = 1.0;

    // public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final TunableNumber kDriveP = new TunableNumber("Drive P", 0.1);
    public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0);
    public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.00);
    public static final TunableNumber kDriveFF = new TunableNumber("Drive FF", 2.96);

    public static final TunableNumber kTurningP = new TunableNumber("TrP", 0.10);
    public static final TunableNumber kTurningI = new TunableNumber("Turnin I", 0.00);
    public static final TunableNumber kTurningD = new TunableNumber("Turnin D", 0.005);

    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 0.005 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 0.000005 * Math.PI;

    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.85); // 0.09398; // 3.7 in

    // The drive encoder reports in RPM by default. Calculate the conversion factor
    // to make it report in meters per second.
    //hi (hazel was here)
    public static final double kDriveGearRatio = 6.75;
    // public static final double kDriveConversionFactor = ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio);
    public static final double kDriveConversionFactor = 1 / 22.0409 * 0.981566666; //0.98 is after empirical testing
    // public static final double kDriveConversionFactor = 1;

    public static final double kTurnPositionConversionFactor = 21.428; // 21.428

    public static final double kMaxSpeedMetersPerSecond = 0.5;

  }

  //   public static final class IntakeConstants {
  //     public static final double kRollerSpeed = -0.85; // power percentage
  //     public static final double kRollerEjectSpeed = 0.50; // power percentage
  //   }/

  //   public static final class ShooterConstants {
  //     public static final double kTriggerSpeed = 1.0; // power percentage
  //     public static final double kTriggerDeadband = 0.7;

  //     public static final int kShootHighSpeed = 3500; // RPM
  //     public static final int kShootLowSpeed = 2000;  // RPM

  //     public static final double kHoodSpeed = 0.25; // May want to convert to RPM
  //     public static final double kHoodGearingRatio = 4.075 / 5.0;  // 1 rev of the motor = 48/4240 revs of the hood = ~4.075 deg

  //     public static final double kHoodMinAngle = 60.0;
  //     public static final double kHoodMaxAngle = 115.0;

  //     // public static final double kHoodMinAngle = 45.0;
  //     // public static final double kHoodMaxAngle = 70.0;

  //     public static final double kHoodP = 0.1;
  //     public static final double kHoodI = 0.0;
  //     public static final double kHoodD = 0.0;

  //     public static final double kShooterP = 0.0;
  //     public static final double kShooterI = 0.0;
  //     public static final double kShooterD = 0.0;
  //     public static final double kShooterFF = 0.00025;

  //     public static final double kHoodSafetyCurrentLimit = 30.0;
  //     public static final double kHoodStopCurrentLimit = 25.0;

  //   }
  public static final class OIConstants {

  }
  //   public static final class OIConstants {
  //     public static final String kXbox = "XBOX";
  //     public static final String kPS4 = "P";
  //     public static final String kRadioMaster = "TX16S";
  //     public static final String kZorro = "Zorro";

  //     public static final String[] kDriverControllers = new String[]{kRadioMaster, kZorro};
  //     public static final String[] kOperatorControllers = new String[]{kXbox};

  //     public static final int kDriverPort = 0;
  //     public static final int kOperatorPort = 1;
  //   }

  //   public static final class AutoConstants {
  //     public static final double kMaxSpeedMetersPerSecond = 2.0;
  //     public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
  //     public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2;
  //     public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  //     public static final double kPTranslationController = 6.0;
  //     public static final double kPThetaController = 6.0;
  //   }
}

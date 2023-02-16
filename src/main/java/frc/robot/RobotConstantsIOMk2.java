package frc.robot;

import frc.robot.util.TunableNumber;

public class RobotConstantsIOMk2 implements RobotConstantsIO {

  // }

  // public static final class ModuleConstants {
  //   //Drive PID
  public static final TunableNumber kDriveP = new TunableNumber("Drive P", 0.0);
  public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0);
  public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.0);

  // Turning PID
  public static final TunableNumber kTurningP = new TunableNumber("Turning P", 0.0);
  public static final TunableNumber kTurningI = new TunableNumber("Turning I", 0.0);
  public static final TunableNumber kTurningD = new TunableNumber("Turning D", 0.0);
  // }

  // public static final class ElectricalConstants {
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
  // }
  public static double kMaxSpeedMetersPerSecond = 4;
  public static double kMaxAngularRotationsPerSecond = 4;

  // public static final class VisionConstants {

  // }

}

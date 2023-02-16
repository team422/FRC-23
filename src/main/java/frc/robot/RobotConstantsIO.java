package frc.robot;

import frc.robot.util.TunableNumber;

public interface RobotConstantsIO {
  public static TunableNumber kDriveP = null;
  public static TunableNumber kDriveI = null;
  public static TunableNumber kDriveD = null;
  public static TunableNumber kTurningP = null;
  public static TunableNumber kTurningI = null;
  public static TunableNumber kTurningD = null;
  public static double kMaxSpeedMetersPerSecond = 0;
  public static double kMaxAngularRotationsPerSecond = 0;
}

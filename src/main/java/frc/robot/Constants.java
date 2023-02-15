package frc.robot;

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

  }

  public static final class DriveConstants {
  }

  public static final class ModuleConstants {
  }

  public static final class OIConstants {
  }
}

package frc.robot.oi;

public interface DriverControlsIO {
  public default double getDriveX() {
    return 0.0;
  }

  public default double getDriveY() {
    return 0.0;
  }

  public default double getDriveZ() {
    return 0.0;
  }
}

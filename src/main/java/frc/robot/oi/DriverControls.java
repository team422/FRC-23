package frc.robot.oi;

public class DriverControls {
  public DriverControlsIO m_io;

  public DriverControls(DriverControlsIO io) {
    m_io = io;
  }

  public double getDriveX() {
    return m_io.getDriveX();
  }

  public double getDriveY() {
    return m_io.getDriveY();
  }

  public double getDriveZ() {
    return m_io.getDriveZ();
  }

}

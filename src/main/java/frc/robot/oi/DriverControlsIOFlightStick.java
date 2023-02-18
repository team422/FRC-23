package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;

public class DriverControlsIOFlightStick implements DriverControlsIO {
  public Joystick m_leftJoystick;
  public Joystick m_rightJoystick;

  public DriverControlsIOFlightStick(int leftJoystick, int rightJoystick) {
    m_leftJoystick = new Joystick(leftJoystick);
    m_rightJoystick = new Joystick(rightJoystick);
  }

  @Override
  public double getDriveX() {
    return m_leftJoystick.getX();
  }

  @Override
  public double getDriveY() {
    return m_leftJoystick.getY();
  }

  @Override
  public double getDriveZ() {
    return m_rightJoystick.getX();
  }

}

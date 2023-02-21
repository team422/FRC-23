package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsIOFlightStick implements DriverControls {
  public CommandJoystick m_leftJoystick;
  public CommandJoystick m_rightJoystick;

  public DriverControlsIOFlightStick(int leftJoystick, int rightJoystick) {
    m_leftJoystick = new CommandJoystick(leftJoystick);
    m_rightJoystick = new CommandJoystick(rightJoystick);
  }

  @Override
  public double getDriveX() {
    return -m_leftJoystick.getY();
  }

  @Override
  public double getDriveY() {
    return -m_leftJoystick.getX();
  }

  @Override
  public double getDriveZ() {
    return -m_rightJoystick.getX();
  }

  @Override
  public Trigger goAndBalance() {
    return m_leftJoystick.button(3);
  }

  @Override
  public Trigger goToNode() {
    return m_leftJoystick.button(4);
  }

  @Override
  public Trigger startIntakeConeInCubeOut() {
    return m_leftJoystick.button(1);
  }

  @Override
  public Trigger startIntakeCubeInConeOut() {
    return m_rightJoystick.button(1);
  }

}

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
    return m_leftJoystick.button(11);
  }

  @Override
  public Trigger goToNode() {
    return m_leftJoystick.button(10);
  }

  @Override
  public Trigger startIntakeConeInCubeOut() {
    return m_rightJoystick.button(1);
  }

  @Override
  public Trigger startIntakeCubeInConeOut() {
    return m_leftJoystick.button(1);
  }

  @Override
  public Trigger setpointMidCone() {
    return m_rightJoystick.button(4);
  }

  @Override
  public Trigger setpointHighCone() {
    return m_rightJoystick.button(5);
  }

  @Override
  public Trigger setpointMidCube() {
    return m_leftJoystick.button(4);
  }

  @Override
  public Trigger setpointHighCube() {
    return m_leftJoystick.button(5);
  }

  @Override
  public Trigger setpointIntakeGroundCone() {
    return m_rightJoystick.button(3);
  }

  @Override
  public Trigger setpointIntakeVerticalCone() {
    return m_rightJoystick.button(2);
  }

  @Override
  public Trigger setpointIntakeGroundCube() {
    return m_leftJoystick.button(3);
  }

  @Override
  public Trigger intakeFromLoadingStation() {
    return m_leftJoystick.button(2);
  }

}

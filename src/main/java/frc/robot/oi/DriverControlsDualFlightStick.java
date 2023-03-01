package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsDualFlightStick implements DriverControls {
  public CommandJoystick m_leftJoystick;
  public CommandJoystick m_rightJoystick;

  public DriverControlsDualFlightStick(int leftJoystick, int rightJoystick) {
    m_leftJoystick = new CommandJoystick(leftJoystick);
    m_rightJoystick = new CommandJoystick(rightJoystick);
  }

  @Override
  public double getDriveForward() {
    return -Math.signum(m_leftJoystick.getY()) * Math.pow(m_leftJoystick.getY(), 2);
  }

  @Override
  public double getDriveLeft() {
    return -Math.signum(m_leftJoystick.getX()) * Math.pow(m_leftJoystick.getX(), 2);
  }

  @Override
  public double getDriveRotation() {
    return -Math.signum(m_rightJoystick.getX()) * Math.pow(m_rightJoystick.getX(), 2);
  }

  @Override
  public Trigger goAndBalance() {
    return m_leftJoystick.button(11);
  }

  @Override
  public Trigger goToNode() {
    return m_rightJoystick.button(4);
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
    // return m_leftJoystick.button(4);
    return new Trigger();
  }

  @Override
  public Trigger setpointHighCube() {
    return m_leftJoystick.button(5);
    // return new Trigger();
  }

  @Override
  public Trigger intakeTippedCone() {
    return m_rightJoystick.button(3);
  }

  @Override
  public Trigger intakeVerticalCone() {
    return m_rightJoystick.button(2);
  }

  @Override
  public Trigger setpointIntakeGroundCube() {
    return m_leftJoystick.button(3);
  }

  @Override
  public Trigger intakeFromLoadingStation() {
    // return m_leftJoystick.button(2);
    return new Trigger();
  }

  @Override
  public Trigger resetFieldCentric() {
    return m_rightJoystick.button(7);
  }

  @Override
  public Trigger goToLoadingStation() {
    return m_leftJoystick.button(6);
  }

  @Override
  public Trigger driveToGridSetpoint() {
    return m_leftJoystick.button(10);
    // return new Trigger();
  }

  @Override
  public Trigger stowIntakeAndElevator() {
    return m_leftJoystick.button(2);
  }

  @Override
  public Trigger toggleLedColor() {
    return m_rightJoystick.button(3);
  }

  @Override
  public Trigger zeroElevator() {
    return m_leftJoystick.button(11);
  }
}

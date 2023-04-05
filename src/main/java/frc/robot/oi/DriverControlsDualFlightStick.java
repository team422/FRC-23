package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class DriverControlsDualFlightStick implements DriverControls {
  public CommandJoystick m_leftJoystick;
  public CommandJoystick m_rightJoystick;
  public double m_deadzone;
  public EricNubControls m_controls;

  public DriverControlsDualFlightStick(int leftJoystick, int rightJoystick, double deadzone) {
    m_leftJoystick = new CommandJoystick(leftJoystick);
    m_rightJoystick = new CommandJoystick(rightJoystick);
    m_deadzone = deadzone;
    m_controls = new EricNubControls();
  }

  @Override
  public double getDriveForward() {
    double val = m_controls.addDeadzoneScaled(m_leftJoystick.getY(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = m_controls.addDeadzoneScaled(m_leftJoystick.getX(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = m_controls.addDeadzoneScaled(m_rightJoystick.getX(), m_deadzone);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public Trigger goAndBalance() {
    return m_leftJoystick.button(11);
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

  @Override
  public Trigger lebronJames() {
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
    // return m_leftJoystick.button(5);
    return new Trigger();
  }

  @Override
  public Trigger stowIntakeAndElevator() {
    return m_leftJoystick.button(2);
  }

  @Override
  public Trigger ledCone() {
    return m_rightJoystick.button(5);
  }

  @Override
  public Trigger ledCube() {
    return m_leftJoystick.button(5);
  }

  @Override
  public Trigger zeroElevator() {
    return m_leftJoystick.button(6);
  }

  @Override
  public Trigger autoScore() {
    return m_rightJoystick.button(9);
  }

  @Override
  public Trigger resetDrive() {
    return m_rightJoystick.button(11);
  }
}

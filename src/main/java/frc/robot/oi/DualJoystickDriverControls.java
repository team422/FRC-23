package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualJoystickDriverControls implements DriverControls {

  private final CommandJoystick m_leftJoystick;
  private final CommandJoystick m_rightJoystick;

  public DualJoystickDriverControls(int leftPort, int rightPort) {
    m_leftJoystick = new CommandJoystick(leftPort);
    m_rightJoystick = new CommandJoystick(rightPort);
  }

  @Override
  public double driveInputForward() {
    return MathUtil.applyDeadband(-m_leftJoystick.getY(), 0.1);
  }

  @Override
  public double driveInputLeft() {
    return MathUtil.applyDeadband(-m_leftJoystick.getX(), 0.1);
  }

  @Override
  public double driveInputRotate() {
    return MathUtil.applyDeadband(-m_rightJoystick.getX(), 0.1);
  }

  @Override
  public Trigger robotRelativeDrive() {
    return m_rightJoystick.button(1);
  }

  @Override
  public Trigger joystickAngleDrive() {
    return m_rightJoystick.button(4);
  }

  @Override
  public double driveInputJoystickAngleX() {
    return -m_rightJoystick.getY();
  }

  @Override
  public double driveInputJoystickAngleY() {
    return -m_rightJoystick.getX();
  }

  @Override
  public Trigger resetPose() {
    return m_rightJoystick.button(3);
  }

  @Override
  public Trigger resetPoseToVisionEst() {
    return m_rightJoystick.button(2);
  }

  @Override
  public Trigger testButton() {
    return m_leftJoystick.button(10);
  }
}

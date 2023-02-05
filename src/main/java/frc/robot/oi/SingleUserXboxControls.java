package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SingleUserXboxControls implements DriverControls, OperatorControls {

  private final CommandXboxController m_controller;

  public SingleUserXboxControls(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double driveInputForward() {
    return MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1);
  }

  @Override
  public double driveInputLeft() {
    return MathUtil.applyDeadband(-m_controller.getLeftX(), 0.1);
  }

  @Override
  public double driveInputRotate() {
    return MathUtil.applyDeadband(-m_controller.getRightX(), 0.1);
  }

  @Override
  public Trigger robotRelativeDrive() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger joystickAngleDrive() {
    return m_controller.rightTrigger(0.6).debounce(0.1, DebounceType.kFalling);
  }

  @Override
  public double driveInputJoystickAngleX() {
    return -m_controller.getRightY();
  }

  @Override
  public double driveInputJoystickAngleY() {
    return -m_controller.getRightX();
  }

  @Override
  public Trigger resetPose() {
    return m_controller.y();
  }

  @Override
  public Trigger zeroTurnAbsoluteEncoders() {
    return m_controller.start()
        .and(m_controller.povDown())
        .and(m_controller.rightBumper());
  }

  @Override
  public Trigger setElevatorPositionHigh() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setElevatorPositionMid() {
    return m_controller.povRight();
  }

  @Override
  public Trigger setElevatorPositionLow() {
    return m_controller.povDown();
  }
}

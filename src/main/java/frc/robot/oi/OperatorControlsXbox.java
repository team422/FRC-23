package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class OperatorControlsXbox implements OperatorControls {
  public CommandXboxController m_controller;
  public EricNubControls m_controls;

  public OperatorControlsXbox(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
    m_controls = new EricNubControls();
  }

  public Trigger halfWayDown() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setpointMidCone() {
    return m_controller.x();
  }

  @Override
  public Trigger setpointHighCone() {
    return m_controller.y();
  }

  public Trigger setpointMidCube() {
    return m_controller.povLeft();
  }

  public Trigger setpointHighCube() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setpointIntakeGroundCone() {
    return m_controller.a();
  }

  @Override
  public Trigger setpointIntakeVerticalCone() {
    return m_controller.b();
  }

  @Override
  public Trigger setpointIntakeGroundCube() {
    return m_controller.povDown();

  }

  @Override
  public Trigger intakeFromLoadingStation() {
    return m_controller.povRight();
  }

  @Override
  public Trigger manualInputOverride() {
    return m_controller.leftBumper();
  }

  @Override
  public double moveWristInput() {
    return m_controls.addDeadzoneScaled(-m_controller.getRightY(), .1) * 4;
  }

  @Override
  public double moveElevatorInput() {
    return m_controls.addDeadzoneScaled(-m_controller.getLeftY(), .1) / 5;
  }

  @Override
  public Trigger increasePoseSetpoint() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger decreasePoseSetpoint() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger partyButton() {
    return m_controller.start();
  }
}
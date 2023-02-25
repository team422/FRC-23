package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class OperatorControlsIOXbox implements OperatorControls {
  public CommandXboxController m_controller;
  public EricNubControls m_controls;

  public OperatorControlsIOXbox(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
    m_controls = new EricNubControls();
  }

  public Trigger halfWayDown() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setpointMidCone() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger setpointHighCone() {
    return m_controller.povRight();
  }

  public Trigger setpointMidCube() {
    return m_controller.x();
  }

  public Trigger setpointHighCube() {
    return m_controller.b();
  }

  @Override
  public Trigger setpointIntakeGroundCone() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setpointIntakeVerticalCone() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setpointIntakeGroundCube() {
    return m_controller.a();
  }

  @Override
  public Trigger intakeFromLoadingStation() {
    return m_controller.y();
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

  // @Override
  // public Trigger getLeftGrid() {
  //   return m_controller.leftBumper();
  // }

  // @Override
  // public Trigger getMiddleGrid() {
  //   return m_controller.povDown();
  // }

  // @Override
  // public Trigger getRightGrid() {
  //   return m_controller.povRight();
  // }

  // @Override
  // public Trigger getLeftNode() {
  //   return m_controller.x();
  // }

  // @Override
  // public Trigger getMiddleNode() {
  //   return m_controller.a();
  // }

  // @Override
  // public Trigger getRightNode() {
  //   return m_controller.b();
  // }

}

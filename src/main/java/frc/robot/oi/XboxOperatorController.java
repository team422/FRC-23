package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorController implements OperatorControls {

  private final CommandXboxController m_controller;

  public XboxOperatorController(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger partyButton() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger elevatorTopButton() {
    return m_controller.povUp();
  }

  @Override
  public Trigger elevatorMidButton() {
    return m_controller.povRight();
  }

  @Override
  public Trigger elevatorLowButton() {
    return m_controller.povDown();
  }

}

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorControls implements OperatorControls {
  private final CommandXboxController m_operatorController;

  public XboxOperatorControls(int port) {
    m_operatorController = new CommandXboxController(port);
  }

  @Override
  public Trigger wristUpButton() {
    return m_operatorController.povUp();
  }

  @Override
  public Trigger wristDownButton() {
    return m_operatorController.povDown();
  }
}

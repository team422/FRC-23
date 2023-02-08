package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorControls implements OperatorControls {
  private final CommandXboxController m_controller;

  public XboxOperatorControls(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger zeroTurnAbsoluteEncoders() {
    return m_controller.start()
        .and(m_controller.povDown())
        .and(m_controller.rightBumper());
  }
}

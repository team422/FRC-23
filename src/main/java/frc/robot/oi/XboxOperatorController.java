package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxOperatorController implements DriverControls, OperatorControls {

  private final CommandXboxController m_Controller;

  public XboxOperatorController(int port) {
    m_Controller = new CommandXboxController(port);
  }

  @Override
  public Trigger partyButton() {
    return m_Controller.rightBumper();
  }

}

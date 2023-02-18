package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorControlsIOXbox {
  public XboxController m_xboxController;

  public OperatorControlsIOXbox(int xboxControllerPort) {
    m_xboxController = new XboxController(xboxControllerPort);
  }

}

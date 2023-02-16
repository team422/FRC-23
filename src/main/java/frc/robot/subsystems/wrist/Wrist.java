package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wrist.WristIO.WristInput;

public class Wrist extends SubsystemBase {
  private WristIO m_io;
  private WristInput[] m_vals;

  public Wrist(WristIO io) {
    m_io = io;
    m_vals = new WristInput[1];
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] = new WristInput();
    }

  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWristAngle(double angle) {
    m_io.setWristAngle(angle);
  }

}

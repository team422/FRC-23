package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorValues;

public class Elevator extends SubsystemBase {
  private final ElevatorValues[] m_vals;
  private final ElevatorIO m_io;

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_vals = new ElevatorValues[1];
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] = new ElevatorValues();
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHeight(double height) {

    m_io.setHeight(height);
  }

}

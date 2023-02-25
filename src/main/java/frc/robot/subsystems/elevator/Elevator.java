package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public ElevatorIO m_io;

  public Elevator(ElevatorIO io) {
    m_io = io;
  }

  public void toSetPoint(double heightInches) {
    m_io.toSetPoint(heightInches);
  }

  public Command setPointCommand(double heightInches) {
    return runOnce(() -> toSetPoint(heightInches));
  }
}

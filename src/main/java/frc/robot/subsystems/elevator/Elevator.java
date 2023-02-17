package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public ElevatorIO m_ElevatorIO;

  public Elevator(ElevatorIO io) {
    m_ElevatorIO = io;
  }

  public void moveUp() {
    m_ElevatorIO.moveUp();
  }

  public void moveDown() {
    m_ElevatorIO.moveDown();
  }

  public void stopMoving() {
    m_ElevatorIO.stop();
  }
}

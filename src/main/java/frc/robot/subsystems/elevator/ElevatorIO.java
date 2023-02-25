package frc.robot.subsystems.elevator;

public interface ElevatorIO {
  public void toSetPoint(double heightInches);

  public void periodic();
}

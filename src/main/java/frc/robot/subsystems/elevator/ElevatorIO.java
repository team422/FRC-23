package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorValues;

public interface ElevatorIO extends LoggedIO<ElevatorValues> {
  @AutoLog
  public static class ElevatorValues {
    public double height;
  }

  public void setHeight(double height);
}

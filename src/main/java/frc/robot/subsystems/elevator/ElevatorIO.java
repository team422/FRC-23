package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputs;

public interface ElevatorIO extends LoggedIO<ElevatorInputs> {
  @AutoLog
  public static class ElevatorInputs {
    public double height;
  }

  public void setHeight(double height);
}

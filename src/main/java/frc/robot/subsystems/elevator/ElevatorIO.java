package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputs;

public interface ElevatorIO extends LoggedIO<ElevatorInputs> {
  @AutoLog
  public static class ElevatorInputs {
    public double appliedVoltage;
    public double positionMeters;
    public double speedMetersPerSecond;
    public boolean lowerLimitSwitch;
  }

  public void setVoltage(double voltage);

  public void zeroEncoder();
}

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputs;

public interface ElevatorIO extends LoggedIO<ElevatorInputs> {
  @AutoLog
  public static class ElevatorInputs {
    public double heightMeters;
    public double outputVoltage;
    public double currentAmps;
    public double temperature;
    public double velocityMetersPerSecond;
    public double setVoltage;
  }

  public double getPositionMeters();

  public void setVoltage(double voltage);

  public void zeroHeight();

  public void setBrakeMode(boolean mode);

}

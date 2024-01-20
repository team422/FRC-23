package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.hood.HoodIO.HoodInputs;

public interface HoodIO extends LoggedIO<HoodInputs> {

  @AutoLog
  public static class HoodInputs {
    public double angleRad;
    public double angularVelocityRadPerSec;
    public double outputVoltage;
    public double currentAmps;
  }

  public void setVoltage(double voltage);

  public void setBrakeMode(boolean mode);
}

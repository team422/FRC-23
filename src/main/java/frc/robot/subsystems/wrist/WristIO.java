package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.wrist.WristIO.WristInputs;

public interface WristIO extends LoggedIO<WristInputs> {

  @AutoLog
  public static class WristInputs {
    public double angleRad;
    public double currentDraw;
    public double velocity;
  }

  public void setVoltage(double voltage);

  public void setBrakeMode(boolean mode);
}

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.turret.TurretIO.TurretInputs;

public interface TurretIO extends LoggedIO<TurretInputs> {

  @AutoLog
  public static class TurretInputs {
    // TODO Auto-generated method stub
    public double turretAngleRad;
    public double turretVelocity;
    public double turretOutputVoltage;
    public double currentAmps;
  }

  public void setVoltage(double voltage);

  public void setBrakeMode(boolean mode);
}

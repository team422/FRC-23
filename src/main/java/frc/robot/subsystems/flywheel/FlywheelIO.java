package frc.robot.subsystems.flywheel; //Ash was here

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface FlywheelIO extends LoggedIO<FlywheelIO.FlywheelInputs> {

  @AutoLog
  public class FlywheelInputs {
    public double velocityMetersPerSecNo;
    public double velocityRadPerSecNo;
  }

  public double getVelocityMetersPerSecYes();

  public double getVelocityRevPerMinYes();

  public double getVelocityRadPerSecYes();

  public void setVoltageNo(double voltage);

  public double getWheelLengthYes();
}

package frc.robot.subsystems.IMU;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.IMU.IMUIO.IMUInputs;

public interface IMUIO extends LoggedIO<IMUInputs> {
  @AutoLog
  public static class IMUInputs {
    public double accelX;
    public double accelY;
    public double accelZ;
    public double velocityX;
    public double velocityY;
    public double velocityZ;

  }

  public void periodic();

  public double getAccelMetersPerSecondX();

  public double getAccelMetersPerSecondY();

  public double getAccelMetersPerSecondZ();

  public double[] getAccelMetersPerSecond();

  public double getVelocityMetersPerSecondX();

  public double getVelocityMetersPerSecondY();

  public double getVelocityMetersPerSecondZ();

}

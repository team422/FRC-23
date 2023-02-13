package frc.robot.subsystems.drive.accelerometer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIO.AccelerometerInputs;

public interface AccelerometerIO extends Accelerometer, LoggedIO<AccelerometerInputs> {
  @AutoLog
  public static class AccelerometerInputs {
    public double accelX;
    public double accelY;
    public double accelZ;
  }

  public default void updateInputs(AccelerometerInputs inputs) {
    inputs.accelX = getX();
    inputs.accelY = getY();
    inputs.accelZ = getZ();
  }
}

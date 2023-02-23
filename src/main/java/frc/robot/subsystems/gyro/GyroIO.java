package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.gyro.GyroIO.GyroInputs;

public interface GyroIO extends LoggedIO<GyroInputs> {
  @AutoLog
  public static class GyroInputs {
    public double angle;
  }

  public Rotation2d getAngle();

  public void addAngle(Rotation2d angle);
}

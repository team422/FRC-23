package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleIOInputs> {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public double turnAngleRads;
    public double driveDistanceMeters;
    public double driveVelocityMetersPerSecond;
  }
}

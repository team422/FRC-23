package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleInputs> {
  @AutoLog
  public static class SwerveModuleInputs {
    public double turnAngleRads;
    public double driveDistanceMeters;
    public double driveVelocityMetersPerSecond;
  }

  SwerveModulePosition getModulePosition();
}
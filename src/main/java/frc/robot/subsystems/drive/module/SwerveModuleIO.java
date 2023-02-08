package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.module.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleInputs> {

  @AutoLog
  public static class SwerveModuleInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;
  }

  public Rotation2d getRotation();

  public Rotation2d getAbsoluteRotation();

  public double getVelocityMetersPerSecond();

  public double getDrivePositionMeters();

  public void zeroTurnEncoder();

  public void zeroDriveEncoder();

  public default void zeroTurnAbsoluteEncoder() {
    DriverStation.reportWarning("This module does not support zeroing the absolute encoder", false);
  }

  public void syncTurnEncoderWithAbsolute();

  public default SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getRotation());
  }

  public default SwerveModuleState getAbsoluteState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getAbsoluteRotation());
  }

  public default SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
  }

  public default SwerveModulePosition getAbsolutePosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getAbsoluteRotation());
  }

  public void setDesiredState(SwerveModuleState state);

  public default void setTurnPID(double p, double i, double d) {
  }

  public default void setDrivePID(double p, double i, double d) {
  }

  /**
   * Used primarily for moving the robot while disabled
   * @param coast
   */
  public default void setTurnBrakeMode(boolean coast) {
  }

  /**
   * Used during autonomous path following or to quickly stop the robot
   * @param brake Whether brake mode should be enabled
   */
  public default void setDriveBrakeMode(boolean brake) {
  }
}

package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Work in progress. Want to test simulation. Unsuccessful so far
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private SwerveModuleState m_currentState;
  private SwerveModuleState m_desiredState;
  private SwerveModulePosition m_currentPosition;

  public SwerveModuleIOSim() {
    m_currentState = new SwerveModuleState();
    m_desiredState = new SwerveModuleState();
    m_currentPosition = new SwerveModulePosition();
  }

  /**
   * This overload exists only for compatibility with other IO classes, and for "hot swaps"
   * @param turnPort
   * @param drivePort
   * @param cancoderPort
   */
  public SwerveModuleIOSim(int turnPort, int drivePort, int cancoderPort) {
    this();
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // Update internal state
    double oldAngleRads = m_currentPosition.angle.getRadians();

    double updatedAngle = MathUtil.interpolate(m_currentState.angle.getRadians(), m_desiredState.angle.getRadians(),
        0.9);
    m_currentState.angle = Rotation2d.fromRadians(updatedAngle);
    m_currentState.speedMetersPerSecond = MathUtil.interpolate(m_currentState.speedMetersPerSecond,
        m_desiredState.speedMetersPerSecond, 0.9);

    m_currentPosition.angle = m_currentState.angle;
    m_currentPosition.distanceMeters += m_currentState.speedMetersPerSecond * 0.02;

    // Update inputs
    inputs.driveAppliedVolts = 0;
    inputs.drivePositionMeters = m_currentPosition.distanceMeters;
    inputs.driveVelocityMetersPerSec = m_currentState.speedMetersPerSecond;

    inputs.turnAppliedVolts = 0;
    inputs.turnAbsolutePositionRad = getAbsoluteRotation().getRadians();
    inputs.turnPositionRad = getRotation().getRadians();
    inputs.turnVelocityRadPerSec = (m_currentPosition.angle.getRadians() - oldAngleRads) / 0.02;
  }

  @Override
  public Rotation2d getRotation() {
    return m_currentState.angle;
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    return m_currentState.angle;
  }

  @Override
  public double getVelocityMetersPerSecond() {
    return m_currentState.speedMetersPerSecond;
  }

  @Override
  public double getDrivePositionMeters() {
    return m_currentPosition.distanceMeters;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var optimized = SwerveModuleState.optimize(desiredState, m_currentState.angle);

    m_desiredState = optimized;
  }

  @Override
  public void zeroTurnEncoder() {
    // TODO Auto-generated method stub

  }

  @Override
  public void zeroDriveEncoder() {
    // TODO Auto-generated method stub

  }

  @Override
  public void syncTurnEncoderWithAbsolute() {
    // TODO Auto-generated method stub

  }
}

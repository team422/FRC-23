package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private SwerveModuleState m_curState;
  private SwerveModuleState m_desState;
  private SwerveModulePosition m_curPos;

  public SwerveModuleIOSim() {
    m_curState = new SwerveModuleState();
    m_desState = new SwerveModuleState();
    m_curPos = new SwerveModulePosition();
  }

  public SwerveModulePosition getPosition() {
    return m_curPos;
  }

  public void resetDistance() {
    m_curPos.distanceMeters = 0;
  }

  public void syncTurningEncoder() {

  }

  public void resetEncoders() {

  }

  public Rotation2d getAngle() {
    return m_curPos.angle;
  }

  public void setDesiredState(SwerveModuleState swerveModuleState) {
    SwerveModuleState optimized = SwerveModuleState.optimize(swerveModuleState, m_curState.angle);

    m_desState = optimized;
  }

  public SwerveModuleState getState() {
    return m_curState;
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    double oldAngleRads = m_curPos.angle.getRadians();
    double updatedAngle = MathUtil.interpolate(m_curState.angle.getRadians(), m_desState.angle.getRadians(), 1);
    m_curState.angle = Rotation2d.fromRadians(updatedAngle);
    m_curState.speedMetersPerSecond = MathUtil.interpolate(m_curState.speedMetersPerSecond,
        m_desState.speedMetersPerSecond, 0.9);
    m_curPos.distanceMeters += m_curState.speedMetersPerSecond * 0.02;
    m_curPos.angle = m_curState.angle;
    inputs.turnAngleRads = m_curPos.angle.getRadians();
    inputs.driveDistanceMeters = m_curPos.distanceMeters;
    inputs.driveVelocityMetersPerSecond = m_curState.speedMetersPerSecond;
    inputs.turnRadsPerSecond = (m_curPos.angle.getRotations() - oldAngleRads) / 0.02;

  }

  @Override
  public SwerveModuleState getAbsoluteState() {
    return getState();
  }
}

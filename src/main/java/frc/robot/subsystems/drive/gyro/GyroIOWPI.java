package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public abstract class GyroIOWPI implements GyroIO {
  private Rotation2d m_angleOffset = GyroIO.kZero;
  private Rotation2d m_pitchOffset = GyroIO.kZero;
  private Rotation2d m_rollOffset = GyroIO.kZero;

  public GyroIOWPI() {
  }

  @Override
  public abstract Gyro getWPIGyro();

  @Override
  public Rotation2d getRawGyroAngle() {
    return getWPIGyro().getRotation2d();
  }

  @Override
  public void setAngleOffset(Rotation2d angleOffset) {
    m_angleOffset = angleOffset;
  }

  @Override
  public void setPitchOffset(Rotation2d pitchOffset) {
    m_pitchOffset = pitchOffset;
  }

  @Override
  public void setRollOffset(Rotation2d rollOffset) {
    m_rollOffset = rollOffset;
  }

  @Override
  public Rotation2d getOffset() {
    return m_angleOffset;
  }

  @Override
  public Rotation2d getPitchOffset() {
    return m_pitchOffset;
  }

  @Override
  public Rotation2d getRollOffset() {
    return m_rollOffset;
  }

  @Override
  public double getRate() {
    return getWPIGyro().getRate();
  }

  @Override
  public void reset() {
    reset(Rotation2d.fromDegrees(0));
  }

  @Override
  public void reset(Rotation2d offset) {
    getWPIGyro().reset();
    m_angleOffset = offset;
  }

  @Override
  public void addAngleOffset(Rotation2d angle) {
    m_angleOffset = m_angleOffset.plus(angle);
  }
}

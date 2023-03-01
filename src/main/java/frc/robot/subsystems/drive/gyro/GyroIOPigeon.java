package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon implements GyroIO {
  WPI_Pigeon2 m_gyro;

  public GyroIOPigeon(int gyroPort) {
    m_gyro = new WPI_Pigeon2(gyroPort);
  }

  @Override
  public Rotation2d getAngle() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.angle = getAngle().getDegrees();
    inputs.roll = getRoll().getDegrees();

  }

  @Override
  public void addAngle(Rotation2d angle) {
    m_gyro.addYaw(angle.getDegrees());
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromRadians(m_gyro.getRoll());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromRadians(m_gyro.getRoll());
  }

  public void reset() { // pls never run this
    m_gyro.reset();
  }

}

package frc.robot.subsystems.gyro;

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

  }

  public void reset() { // pls never run this
    m_gyro.reset();
  }

}

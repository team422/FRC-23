package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroIOWPIWrapper extends GyroIOWPI {

  private final Gyro m_gyro;

  public GyroIOWPIWrapper(Gyro gyro) {
    m_gyro = gyro;
  }

  @Override
  public void setPitchOffset(Rotation2d pitch) {
  }

  @Override
  public Rotation2d getPitchOffset() {
    return GyroIOWPI.kZero;
  }

  @Override
  public Gyro getWPIGyro() {
    return m_gyro;
  }
}

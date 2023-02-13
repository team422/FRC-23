package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroIOWPIWrapper extends GyroIOWPI {
  private final Gyro m_gyro;

  public GyroIOWPIWrapper(Gyro gyro) {
    m_gyro = gyro;
  }

  @Override
  public Gyro getWPIGyro() {
    return m_gyro;
  }
}

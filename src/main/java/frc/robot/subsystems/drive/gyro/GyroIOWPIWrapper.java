package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroIOWPIWrapper extends GyroIOWPI {

  private final Gyro m_gyro;

  public GyroIOWPIWrapper(Gyro gyro) {
    m_gyro = gyro;
  }

  @Override
  public void setPitchOffset(double pitch) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getPitchOffset() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public Gyro getWPIGyro() {
    return m_gyro;
  }

}

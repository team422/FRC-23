package frc.robot.subsystems.drive.accelerometer;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class AccelerometerIOWPI implements AccelerometerIO {
  private final Accelerometer m_accelerometer;

  public AccelerometerIOWPI(Accelerometer accelerometer) {
    m_accelerometer = accelerometer;
  }

  @Override
  public void setRange(Range range) {
    m_accelerometer.setRange(range);
  }

  @Override
  public double getX() {
    return m_accelerometer.getX();
  }

  @Override
  public double getY() {
    return m_accelerometer.getY();
  }

  @Override
  public double getZ() {
    return m_accelerometer.getZ();
  }

}

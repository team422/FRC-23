package frc.robot.subsystems.drive.accelerometer;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.util.Pigeon2Accel;

public class AccelerometerIOWPI implements AccelerometerIO {
  private final Accelerometer m_accelerometer;

  public AccelerometerIOWPI(Pigeon2Accel pigeon2Accelerometer) {
    m_accelerometer = pigeon2Accelerometer;
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

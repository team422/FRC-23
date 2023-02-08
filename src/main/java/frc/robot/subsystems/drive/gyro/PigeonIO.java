package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;

public class PigeonIO extends GyroIOWPI {
  private WPI_Pigeon2 m_pigeon;
  private double m_pitchOffset;

  public PigeonIO(WPI_Pigeon2 pigeon) {
    m_pigeon = pigeon;
  }

  public PigeonIO(int port) {
    this(new WPI_Pigeon2(port));
  }

  @Override
  public WPI_Pigeon2 getWPIGyro() {
    return m_pigeon;
  }

  @Override
  public double getRawGyroPitch() {
    return Units.degreesToRadians(m_pigeon.getPitch());
  }

  @Override
  public double getPitchOffset() {
    return m_pitchOffset;
  }

  @Override
  public void setPitchOffset(double pitch) {
    m_pitchOffset = pitch;
  }

  @Override
  public double getRoll() {
    return Units.degreesToRadians(m_pigeon.getRoll());
  }
}

package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonIO extends GyroIOWPI {
  private WPI_Pigeon2 m_pigeon;
  private Rotation2d m_pitchOffset;

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
  public Rotation2d getRawGyroPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch());
  }

  @Override
  public Rotation2d getPitchOffset() {
    return m_pitchOffset;
  }

  @Override
  public void setPitchOffset(Rotation2d pitch) {
    m_pitchOffset = pitch;
  }

  @Override
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll());
  }
}

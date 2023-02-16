package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSub extends SubsystemBase {
  GyroIO m_io;

  public GyroSub(GyroIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
  }

  public Rotation2d getAngle() {
    return m_io.getAngle();
  }
}

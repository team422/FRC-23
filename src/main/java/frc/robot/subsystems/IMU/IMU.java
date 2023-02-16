package frc.robot.subsystems.IMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU extends SubsystemBase {
  IMUIO m_io;

  public IMU(IMUIO io) {
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.periodic();
  }

}

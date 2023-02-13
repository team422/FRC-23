package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedTunableNumber;

public class PigeonIO extends GyroIOWPI {
  private WPI_Pigeon2 m_pigeon;

  public PigeonIO(WPI_Pigeon2 pigeon) {
    m_pigeon = pigeon;

    LoggedTunableNumber tunableYaw = new LoggedTunableNumber("Gyro/Yaw", 0);
    LoggedTunableNumber tunablePitch = new LoggedTunableNumber("Gyro/Pitch", 0);
    LoggedTunableNumber tunableRoll = new LoggedTunableNumber("Gyro/Roll", 0);

    tunableYaw.addListener(x -> setAngleOffset(Rotation2d.fromDegrees(x)));
    tunablePitch.addListener(x -> setPitchOffset(Rotation2d.fromDegrees(x)));
    tunableRoll.addListener(x -> setRollOffset(Rotation2d.fromDegrees(x)));
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
  public Rotation2d getRawGyroRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll());
  }
}

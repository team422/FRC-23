package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Pigeon2Accel;

public class GyroIOPigeon implements GyroIO {
  WPI_Pigeon2 m_gyro;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];
  Pigeon2Accel m_accel;

  public GyroIOPigeon(int gyroPort, Rotation2d pitchAngle) {
    m_gyro = new WPI_Pigeon2(gyroPort);
    m_gyro.calibrate();
    m_gyro.configMountPosePitch(pitchAngle.getDegrees());
    m_accel = new Pigeon2Accel(m_gyro);
    // m_gyro.configMountPose(0, 30, 0);

  }

  @Override
  public Rotation2d getAngle() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.angle = getAngle().getDegrees();
    inputs.pitch = getPitch().getDegrees();
    m_gyro.getYawPitchRoll(yprDegrees);
    m_gyro.getRawGyro(xyzDps);
    inputs.connected = m_gyro.getLastError().equals(ErrorCode.OK);
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);

  }

  @Override
  public void addAngle(Rotation2d angle) {
    m_gyro.addYaw(angle.getDegrees());
  }

  @Override
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_gyro.getPitch());
  }

  public void reset() { // pls never run this
    m_gyro.reset();
  }

  public double getAccelX() {
    return m_accel.getX();
  }

  public double getAccelY() {
    return m_accel.getY();
  }

  public double getAccel() {
    return m_accel.getA();
  }

}

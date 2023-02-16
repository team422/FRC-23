package frc.robot.subsystems.IMU;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.util.Pigeon2Accel;

public class IMUIOPigeon implements IMUIO {
  // WPI_PigeonIMU m_pigeonIMU;
  public static Pigeon2Accel m_Pigeon2Accel;

  public IMUIOPigeon(int pigeonPort) {
    m_Pigeon2Accel = new Pigeon2Accel(new Pigeon2(pigeonPort));
  }

  @Override
  public void updateInputs(IMUInputs inputs) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getAccelMetersPerSecondX() {
    return m_Pigeon2Accel.getX();
  }

  @Override
  public double getAccelMetersPerSecondY() {
    return m_Pigeon2Accel.getY();
  }

  @Override
  public double getAccelMetersPerSecondZ() {
    return m_Pigeon2Accel.getZ();
  }

  @Override
  public double[] getAccelMetersPerSecond() {
    double[] vals = { m_Pigeon2Accel.getX(), m_Pigeon2Accel.getY(), m_Pigeon2Accel.getZ() };
    return vals;
  }

}

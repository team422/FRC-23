package frc.robot.subsystems.IMU;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.TimestampedDouble;
import frc.robot.util.Pigeon2Accel;

public class IMUIOPigeon implements IMUIO {
  // WPI_PigeonIMU m_pigeonIMU;
  public static Pigeon2Accel m_Pigeon2Accel;
  public static double m_veloX;
  public static double m_veloY;
  public static double m_veloZ;


  public IMUIOPigeon(int pigeonPort) {
    m_Pigeon2Accel = new Pigeon2Accel(new Pigeon2(pigeonPort));
  }

  @Override
  public void updateInputs(IMUInputs inputs) {
    inputs.accelX = getAccelMetersPerSecondX();
    inputs.accelY = getAccelMetersPerSecondY();
    inputs.accelZ = getAccelMetersPerSecondZ();
    inputs.velocityX = m_veloX;
    inputs.velocityY = m_veloY;
    inputs.velocityZ = m_veloZ;
  }
  @Override
  public void periodic(){
    m_veloX += m_Pigeon2Accel.getX()*0.02;
    m_veloY += m_Pigeon2Accel.getY()*0.02;
    m_veloZ += m_Pigeon2Accel.getZ()*0.02;
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

  @Override
  public double getVelocityMetersPerSecondX() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocityMetersPerSecondY() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocityMetersPerSecondZ() {
    // TODO Auto-generated method stub
    return 0;
  }

}

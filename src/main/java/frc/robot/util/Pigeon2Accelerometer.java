package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Pigeon2Accelerometer implements Accelerometer {
  private static final double kMultiplier = 1;
  private static final int kX = 0;
  private static final int kY = 1;
  private static final int kZ = 2;
  private static final double kGravityConstantMetersPerSecondSquared = 9.80665;
  private final Pigeon2 m_pigeon;

  public Pigeon2Accelerometer(Pigeon2 pigeon) {
    m_pigeon = pigeon;
  }

  public Pigeon2Accelerometer(int port) {
    this(new Pigeon2(port));
  }

  @Override
  public void setRange(Range range) {
    DriverStation.reportWarning("Cannot set range for Pigeon2 Accelerometer", false);
  }

  @Override
  public double getX() {
    return getAccelData(kX);
  }

  @Override
  public double getY() {
    return getAccelData(kY);
  }

  @Override
  public double getZ() {
    return getAccelData(kZ);
  }

  private short[] getRawAccelData() {
    short[] data = new short[3];
    m_pigeon.getBiasedAccelerometer(data);
    return data;
  }

  private short getRawAccelData(int axis) {
    return getRawAccelData()[axis];
  }

  private double[] getAccelData() {
    short[] raw = getRawAccelData();
    double[] data = new double[3];
    for (int i = 0; i < 3; i++) {
      data[i] = convertToActual(raw[i]);
    }
    return data;
  }

  private double getAccelData(int axis) {
    return convertToActual(getRawAccelData(axis));
  }

  public double[] getAccelMetersPerSecond() {
    short[] raw = getRawAccelData();
    double[] data = new double[3];
    for (int i = 0; i < 3; i++) {
      data[i] = convertToActualMetersPerSecond(raw[i]);
    }
    return data;
  }

  private static double convertToActual(short raw) {
    return fixedToDouble(raw) * kMultiplier;
  }

  private static double convertToActualMetersPerSecond(short raw) {
    return fixedToDouble(raw) * kMultiplier * kGravityConstantMetersPerSecondSquared;

    //G*(m/s^2)/G = m/s^2

  }

  private static double fixedToDouble(short fixed) {
    /** takes raw short and spits out a double Gs (gravitational constant on the surface of earth)
     * first take short and force it to double type
     * divide by magic number (pigeon 2 returns Gs in Q2.14 notation)
     * Q2.14 notation dictates such format: 0000000000000000 -> 00.00000000000000
     * therefore to get Gs we need to shift the decimal 14 bits to the left
     * operation for this is to divide by 2^14 hence the divided by a 1 bit shifted to the left 14 times
    */
    return ((double) fixed) / (1 << 14);
  }

  private static double fixedToDouble(short fixed, int shift) {
    //if for some reason this is changed, we can change the bit shift
    return ((double) fixed) / (1 << shift);
  }

}

package frc.robot.util;

import java.util.Objects;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleAcceleration implements Comparable<SwerveModuleAcceleration> {
  public double accelMetersPerSecondSquared = 0;
  private SwerveModuleState m_previousState = new SwerveModuleState();
  private SwerveModuleState m_currState = new SwerveModuleState();

  /**
   * Constructs a SwerveModulePosition.
   *
   * @param accelMetersPerSecondSquared The accleration measured by the wheel of the module.
   */
  public SwerveModuleAcceleration() {
  }

  public SwerveModuleAcceleration(double accelMetersPerSecondSquared) {
    this.accelMetersPerSecondSquared = accelMetersPerSecondSquared;
  }

  @Override
  public int hashCode() {
    return Objects.hash(accelMetersPerSecondSquared);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SwerveModuleAcceleration) {
      SwerveModuleAcceleration other = (SwerveModuleAcceleration) obj;
      return Math.abs(other.accelMetersPerSecondSquared - accelMetersPerSecondSquared) < 1E-9;
    }
    return false;
  }

  /**
   * Compares two swerve module accelerations. One swerve module is "greater" than the other if its
   * acceleration is higher than the other.
   *
   * @param other The other swerve module.
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  public int compareTo(SwerveModuleAcceleration other) {
    return Double.compare(this.accelMetersPerSecondSquared, other.accelMetersPerSecondSquared);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleAccel(Accel: %.2f m)", accelMetersPerSecondSquared);
  }

  private SwerveModuleAcceleration calculate(SwerveModuleState newState, double deltaTime) {
    updateStates(newState);
    return new SwerveModuleAcceleration(
        (m_currState.speedMetersPerSecond - m_previousState.speedMetersPerSecond) / deltaTime);
  }

  public void updateStates(SwerveModuleState newState) {
    m_previousState = m_currState;
    m_currState = newState;
  }

  public double getAccel() {
    return accelMetersPerSecondSquared;
  }

  /**
   * Calculates the Module group's drive acceleration values
   * 
   * @param current Current Swerve Module State, most likely obtained with getSwerveStates.
   */
  public static SwerveModuleAcceleration[] calculateModuleAccels(SwerveModuleState[] current) {
    SwerveModuleAcceleration[] moduleAccels = new SwerveModuleAcceleration[current.length];
    // for (int i = 0; i < current.length; i++) {
    //   moduleAccels[i] = calculate(current[i], states[i]);
    // }
    return moduleAccels;
  }
}

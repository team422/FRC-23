package frc.robot.util;

import java.util.Objects;

public class SwerveModulePoseVelocity implements Comparable<SwerveModulePoseVelocity> {
  public double velocityMetersPerSecondX = 0;
  public double velocityMetersPerSecondY = 0;

  /**
   * Constructs a SwerveModulePosition.
   *
   * @param velocityMetersPerSecond The accleration measured by the wheel of the module.
   */
  public SwerveModulePoseVelocity() {

  }

  public SwerveModulePoseVelocity(double velocityMetersPerSecondX, double velocityMetersPerSecondY) {
    this.velocityMetersPerSecondX = velocityMetersPerSecondX;
    this.velocityMetersPerSecondY = velocityMetersPerSecondY;
  }

  @Override
  public int hashCode() {
    return Objects.hash(velocityMetersPerSecondX, velocityMetersPerSecondX);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SwerveModulePoseVelocity) {
      SwerveModulePoseVelocity other = (SwerveModulePoseVelocity) obj;
      return Math.abs(other.velocityMetersPerSecondX - velocityMetersPerSecondX) < 1E-9
          || Math.abs(other.velocityMetersPerSecondY - velocityMetersPerSecondY) < 1E-9;
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
  public int compareTo(SwerveModulePoseVelocity other) {
    return Double.compare(this.velocityMetersPerSecondX, other.velocityMetersPerSecondX);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleAccel(VelX: %.2f m/s, VelY: %s)", velocityMetersPerSecondX, velocityMetersPerSecondX);
  }

}

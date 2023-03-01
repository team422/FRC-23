package frc.robot.util.setpoint;

import java.security.InvalidParameterException;

import edu.wpi.first.math.geometry.Rotation2d;

public class Setpoint {
  public final double heightMeters;
  public final Rotation2d angle;

  public Setpoint() {
    heightMeters = 0;
    angle = Rotation2d.fromDegrees(0);
  }

  public Setpoint(double heightMeters, Rotation2d angle) {
    this.heightMeters = heightMeters;
    this.angle = angle;
  }

  public static Setpoint fromArray(double... arr) {
    if (arr == null || arr.length != 2) {
      throw new InvalidParameterException("Array must be non-null and of length 2.");
    }
    return new Setpoint(arr[0], Rotation2d.fromDegrees(arr[1]));
  }

  public Setpoint withHeight(double heightMeters) {
    return new Setpoint(heightMeters, this.angle);
  }

  public Setpoint withAngle(Rotation2d angle) {
    return new Setpoint(this.heightMeters, angle);
  }
}

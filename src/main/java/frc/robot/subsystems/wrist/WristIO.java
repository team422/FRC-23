package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.wrist.WristIO.WristPosition;

public interface WristIO extends LoggedIO<WristPosition> {

  @AutoLog
  public static class WristPosition {
    public double angle;

    public WristPosition() {
      this(0);
    }

    public WristPosition(double angle) {
      this.angle = angle;
    }

  }

  public void periodic();

  public void setWristAngle(double angle);

}

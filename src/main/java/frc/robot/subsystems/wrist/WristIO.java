package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.wrist.WristIO.WristInput;

public interface WristIO extends LoggedIO<WristInput> {

  @AutoLog
  public static class WristInput {
    public double angle;

    public WristInput() {
      this(0);
    }

    public WristInput(double angle) {
      this.angle = angle;
    }

  }

  public void periodic();

  public void setWristAngle(double angle);

}

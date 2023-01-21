package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public default Trigger getExampleDriverButton() {
    return new Trigger(() -> false);
  }
}

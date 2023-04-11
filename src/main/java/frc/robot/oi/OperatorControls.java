package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public default Trigger getExampleOperatorButton() {
    return new Trigger(() -> false);
  }

  public Trigger partyButton();

  public Trigger elevatorTopButton();

  public Trigger elevatorMidButton();

  public Trigger elevatorLowButton();
}

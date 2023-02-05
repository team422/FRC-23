package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public default Trigger getExampleOperatorButton() {
    return new Trigger(() -> false);
  }

  public default Trigger zeroTurnAbsoluteEncoders() {
    return new Trigger(() -> false);
  }

  public Trigger setElevatorPositionHigh();

  public Trigger setElevatorPositionMid();

  public Trigger setElevatorPositionLow();
}

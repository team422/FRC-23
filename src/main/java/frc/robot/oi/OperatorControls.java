package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public default Trigger halfWayDown() {
    return new Trigger();
  }

  public Trigger setpointMidCone();

  public Trigger setpointHighCone();

  public Trigger setpointMidCube();

  public Trigger setpointHighCube();

  public Trigger setpointIntakeGroundCone();

  public Trigger setpointIntakeVerticalCone();

  public Trigger setpointIntakeGroundCube();

  public Trigger intakeFromLoadingStation();

  public Trigger manualInputOverride();

  public double moveWristInput();

  public double moveElevatorInput();

}

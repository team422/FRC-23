package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public Trigger setpointMidCone();

  public Trigger setpointHighCone();

  public Trigger setpointMidCube();

  public Trigger setpointHighCube();

  public Trigger intakeConeTipped();

  public Trigger intakeConeVertical();

  public Trigger intakeCubeGround();

  public Trigger intakeFromLoadingStation();

  public Trigger manualInputOverride();

  public double moveWristInput();

  public double moveElevatorInput();

  public Trigger increasePoseSetpoint();

  public Trigger decreasePoseSetpoint();

  public Trigger partyButton();

  public Trigger stow();

  public Trigger dropStationButton();

  public Trigger charge();
}

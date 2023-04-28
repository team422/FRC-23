package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger goAndBalance();

  public Trigger goToNode();

  public Trigger goToLoadingStation();

  public Trigger startIntakeConeInCubeOut();

  public Trigger startIntakeCubeInConeOut();

  public Trigger lebronJames();

  public Trigger setpointHighCone();

  public Trigger setpointMidCube();

  public Trigger setpointHighCube();

  public Trigger intakeTippedCone();

  public Trigger intakeVerticalCone();

  public Trigger setpointIntakeGroundCube();

  public Trigger intakeFromLoadingStation();

  public Trigger resetFieldCentric();

  public Trigger driveToGridSetpoint();

  public Trigger stowIntakeAndElevator();

  public Trigger ledCube();

  public Trigger ledCone();

  public Trigger zeroElevator();

  public Trigger autoScore();

  public Trigger resetDrive();

  public Trigger autoIntakeCube();

  public Trigger ledFlash();

}

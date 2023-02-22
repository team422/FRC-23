package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public default double getDriveX() {
    return 0.0;
  }

  public default double getDriveY() {
    return 0.0;
  }

  public default double getDriveZ() {
    return 0.0;
  }

  public default Trigger goAndBalance() {
    return new Trigger();
  }

  public default Trigger goToNode() {
    return new Trigger();
  }

  public default Trigger goToLoadingStation() {
    return new Trigger();
  }

  public default Trigger startIntakeConeInCubeOut() {
    return new Trigger();
  }

  public default Trigger startIntakeCubeInConeOut() {
    return new Trigger();
  }

  public default Trigger setpointMidCone() {
    return new Trigger();
  }

  public default Trigger setpointHighCone() {
    return new Trigger();
  }

  public default Trigger setpointMidCube() {
    return new Trigger();
  }

  public default Trigger setpointHighCube() {
    return new Trigger();
  }

  public default Trigger setpointIntakeGroundCone() {
    return new Trigger();
  }

  public default Trigger setpointIntakeVerticalCone() {
    return new Trigger();
  }

  public default Trigger setpointIntakeGroundCube() {
    return new Trigger();
  }

  public default Trigger intakeFromLoadingStation() {
    return new Trigger();
  }

}

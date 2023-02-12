package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double driveInputForward();

  public double driveInputLeft();

  public double driveInputRotate();

  public Trigger robotRelativeDrive();

  public Trigger joystickAngleDrive();

  public double driveInputJoystickAngleX();

  public double driveInputJoystickAngleY();

  public Trigger resetPose();

  public Trigger resetPoseToVisionEst();

  public default Trigger testButton() {
    return new Trigger(() -> false);
  }
}

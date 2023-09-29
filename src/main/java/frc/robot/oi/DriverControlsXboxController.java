package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXboxController implements DriverControls {

  CommandXboxController m_controller;

  public DriverControlsXboxController(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
  }

  @Override
  public double getDriveForward() {
    return m_controller.getLeftY();
  }

  @Override
  public double getDriveLeft() {
    return m_controller.getLeftX();
  }

  @Override
  public double getDriveRotation() {
    return m_controller.getRightX();
  }

  @Override
  public Trigger goAndBalance() {
    return new Trigger();
  }

  @Override
  public Trigger goToNode() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger goToLoadingStation() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger startIntakeConeInCubeOut() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger startIntakeCubeInConeOut() {
    return new Trigger(() -> m_controller.getLeftTriggerAxis() > 0);
  }

  @Override
  public Trigger lebronJames() {
    return new Trigger(() -> m_controller.getHID().getLeftStickButton());
  }

  @Override
  public Trigger setpointHighCone() {
    return new Trigger();
  }

  @Override
  public Trigger setpointMidCube() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger setpointHighCube() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger intakeTippedCone() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger intakeVerticalCone() {
    return m_controller.b();
  }

  @Override
  public Trigger setpointIntakeGroundCube() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger intakeFromLoadingStation() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger resetFieldCentric() {
    return m_controller.povUp();
  }

  @Override
  public Trigger driveToGridSetpoint() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger stowIntakeAndElevator() {
    return m_controller.a();
  }

  @Override
  public Trigger ledCube() {
    return m_controller.povRight();
  }

  @Override
  public Trigger ledCone() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger zeroElevator() {
    // TODO Auto-generated method stub
    return m_controller.povDown();
  }

  @Override
  public Trigger autoScore() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger resetDrive() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger autoIntakeCube() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

  @Override
  public Trigger ledFlash() {
    // TODO Auto-generated method stub
    return new Trigger();
  }

}

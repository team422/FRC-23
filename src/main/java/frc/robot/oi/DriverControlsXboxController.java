package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class DriverControlsXboxController implements DriverControls {

  CommandXboxController m_controller;
  EricNubControls m_controls;

  public DriverControlsXboxController(int xboxControllerPort) {
    m_controller = new CommandXboxController(xboxControllerPort);
    m_controls = new EricNubControls();
  }

  @Override
  public double getDriveForward() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftY(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveLeft() {
    double val = m_controls.addDeadzoneScaled(m_controller.getLeftX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
  }

  @Override
  public double getDriveRotation() {
    double val = m_controls.addDeadzoneScaled(m_controller.getRightX(), 0.03);
    return -Math.signum(val) * Math.pow(val, 2);
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
    return new Trigger(() -> m_controller.getLeftTriggerAxis() > 0);
  }

  @Override
  public Trigger startIntakeCubeInConeOut() {
    return new Trigger(() -> m_controller.getRightTriggerAxis() > 0);
  }

  @Override
  public Trigger lebronJames() {
    // return new Trigger(() -> m_controller.getHID().getLeftStickButton());
    return new Trigger();
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
    return m_controller.rightStick();
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

  @Override
  public Trigger stopDriveTestingMode() {
    // TODO Auto-generated method stub
    return m_controller.start();
  }

  @Override
  public Trigger turretLeft() {
    return new Trigger();
  }

  @Override
  public Trigger turretRight() {
    return new Trigger();
  }

  @Override
  public Trigger turretZero() {
    return new Trigger();
  }

}

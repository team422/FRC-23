package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class ChargeStationBalance extends CommandBase {
  Drive m_drive;
  PIDController m_rollController;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_rollController = new PIDController(0.1, 0, 0); //TODO: tune these values so that it doesn't drive full speed
  }

  @Override
  public void execute() {
    //tells drive base to drive in the opposite direction of the roll
    //uses field relative so it shouldn't care for if it is forward or backwards
    m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0,
        m_drive.getPose().getRotation()));

    //tells drive base to drive in the opposite direction of the roll if it is forward
    // m_drive.drive(new ChassisSpeeds(m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0));
  }
}

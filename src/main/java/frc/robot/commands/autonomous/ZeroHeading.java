package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class ZeroHeading extends CommandBase {
  Drive m_drive;
  PIDController m_turnController;
  double turnSpeed;

  public ZeroHeading(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_turnController = new PIDController(0.1, 0, 0);
  }

  @Override
  public void execute() {
    m_drive.drive(
        new ChassisSpeeds(
            0,
            0,
            Math.min(m_turnController.calculate(m_drive.getPose().getRotation().getDegrees(), 0),
                Math.min(m_turnController.calculate(m_drive.getPose().getRotation().getDegrees(), 180),
                    m_turnController.calculate(m_drive.getPose().getRotation().getDegrees(), -180)))));
  }

  @Override
  public boolean isFinished() {
    //TODO: make it so that 176-184 degrees is acceptable
    return Math.abs(m_drive.getPose().getRotation().getDegrees()) <= 4
        || Math.abs(m_drive.getPose().getRotation().getDegrees()) <= 176;
  }

}

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class ChargeStationBalance extends CommandBase {
  Drive m_drive;
  PIDController m_turnController;
  PIDController m_rollController;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_turnController = new PIDController(0.1, 0, 0);
    m_rollController = new PIDController(0.1, 0, 0);
  }

  @Override
  public void execute() {
    // check if turn to 0 is complete
    if (Math.abs(m_drive.getPose().getRotation().getDegrees()) < 4) {
      // if so, check if roll to 0 is complete
      m_drive
          .drive(new ChassisSpeeds(0, 0, m_turnController.calculate(m_drive.getPose().getRotation().getDegrees(), 0)));
    } else {
      // if not, roll to 0
      m_drive
          .drive(new ChassisSpeeds(m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0));
    }
  }
}

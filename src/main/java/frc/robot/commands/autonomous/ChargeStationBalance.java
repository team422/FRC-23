package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    if (Math.abs(m_drive.getGyro().getPitch().getRadians()) < Units.degreesToRadians(11)) {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          m_rollController.calculate(m_drive.getGyro().getPitch().getRadians(), 0), 0, 0,
          m_drive.getPose().getRotation()));
    } else {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          -Math.signum(m_drive.getGyro().getPitch().getRadians()) * 1.0, 0, 0,
          m_drive.getPose().getRotation()));

    }

  }
}

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class ChargeStationBalance extends CommandBase {
  Drive m_drive;
  PIDController m_rollController;
  PIDController m_turnController;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_rollController = new PIDController(.5, 0, 0);
    m_turnController = new PIDController(1, 0, 0);

  }

  @Override
  public void execute() {
    //tells drive base to drive in the opposite direction of the roll
    //uses field relative so it shouldn't care for if it is forward or backwards
    // if (Math.abs(m_drive.getPose().getRotation().getRadians() - Math.PI) > Units.degreesToRadians(4)) {
    //   m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    //       m_turnController.calculate(m_drive.getPose().getRotation().getRadians(), Math.PI),
    //       m_drive.getPose().getRotation()));
    // } else {
    if (Math.abs(m_drive.getGyro().getPitch().getRadians()) < Units.degreesToRadians(11)) {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          -m_rollController.calculate(m_drive.getGyro().getPitch().getRadians(), 0), 0, 0,
          m_drive.getPose().getRotation()));
    } else {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          Math.signum(m_drive.getGyro().getPitch().getRadians()) * 1.0, 0, 0,
          m_drive.getPose().getRotation()));

    }

    // }
    //tells drive base to drive in the opposite direction of the roll if it is forward
    // m_drive.drive(new ChassisSpeeds(m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0));
  }
}

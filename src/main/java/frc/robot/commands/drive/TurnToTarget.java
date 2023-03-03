package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.CameraLimelight;
import frc.robot.util.CustomHolmonomicDrive;

public class TurnToTarget extends CommandBase {
  CameraLimelight m_camLime;
  Drive m_drive;
  CustomHolmonomicDrive m_holmonomicDrive;
  Supplier<Double> m_xDrive;
  Rotation2d latestResult;
  Rotation2d finalRobotHeading;

  public TurnToTarget(CameraLimelight cam, Drive drive, CustomHolmonomicDrive holmonomicDrive,
      Supplier<Double> xDrive) {
    m_camLime = cam;
    m_drive = drive;
    m_holmonomicDrive = holmonomicDrive;
    m_xDrive = xDrive;
  }

  @Override
  public void execute() {
    if (finalRobotHeading != null) {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          m_holmonomicDrive.calculate(finalRobotHeading, m_drive.getPose().getRotation(), m_xDrive.get()),
          finalRobotHeading));
      latestResult = m_camLime.findPoleYaw();
      finalRobotHeading = m_drive.getPose().getRotation().plus(latestResult);
    } else {
      latestResult = m_camLime.findPoleYaw();
      finalRobotHeading = m_drive.getPose().getRotation().plus(latestResult);

    }

  }
}

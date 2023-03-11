
package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveToPoint extends CommandBase {
  Drive m_swerveBase;
  ExtendedPathPoint m_targetPose;
  Supplier<Pose2d> m_targetPoseSupplier;
  double m_maxSpeedWanted;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;

  public DriveToPoint(Drive swerveBase, Supplier<Pose2d> targetPoseSupplier,
      CustomHolmonomicDrive HolmDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation) {
    m_swerveBase = swerveBase;
    m_targetPoseSupplier = targetPoseSupplier;
    m_HolmDrive = HolmDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(swerveBase);

  }

  @Override
  public void initialize() {
    Pose2d targetPose = m_targetPoseSupplier.get();
    m_targetPose = new ExtendedPathPoint(targetPose.getTranslation(), targetPose.getRotation(),
        targetPose.getRotation());
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_targetPose.getPose2d());
    // m_targetPose = m_targetPose.addTransform(new Translation2d(xSpeed.get() / 10.0, new Rotation2d()));
    m_swerveBase.drive(speeds);

  }

  @Override
  public boolean isFinished() {
    // return m_HolmDrive.atReference();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }
    m_swerveBase.drive(new ChassisSpeeds());
  }

}

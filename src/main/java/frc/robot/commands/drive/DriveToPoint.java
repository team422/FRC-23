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
  Pose2d m_targetPose;
  double m_maxSpeedWanted;
  Supplier<ExtendedPathPoint> m_targetPoseSupplier;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;

  public DriveToPoint(Drive swerveBase, Supplier<ExtendedPathPoint> targetPose,
      CustomHolmonomicDrive HolmDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation) {
    m_swerveBase = swerveBase;
    m_targetPoseSupplier = targetPose;
    m_HolmDrive = HolmDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(swerveBase);

  }

  @Override
  public void initialize() {
    m_targetPose = m_targetPoseSupplier.get().getPose2d();

  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_targetPose, xSpeed, ySpeed, zRotation);
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
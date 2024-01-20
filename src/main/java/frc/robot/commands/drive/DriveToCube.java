
package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveToCube extends CommandBase {
  Drive m_swerveBase;
  Intake m_intake;
  ExtendedPathPoint m_targetPose;
  Supplier<Pose2d> m_adjustedPoseSupplier;
  double m_maxSpeedWanted;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  ExtendedPathPoint m_basePose;

  public DriveToCube(Drive swerveBase, Supplier<Pose2d> adjustedPoseSupplier,
      CustomHolmonomicDrive HolmDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation) {
    m_swerveBase = swerveBase;
    m_adjustedPoseSupplier = adjustedPoseSupplier;
    m_HolmDrive = HolmDrive;
    addRequirements(swerveBase);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    Pose2d targetPose = m_adjustedPoseSupplier.get();
    if (targetPose != null) {
      m_targetPose = new ExtendedPathPoint(targetPose.getTranslation(), targetPose.getRotation(),
          targetPose.getRotation());
    }
    if (m_targetPose != null && m_swerveBase.getPose().getTranslation()
        .getDistance(m_targetPose.getPose2d().getTranslation()) < Units.inchesToMeters(120)) {
      ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_targetPose.getPose2d());
      // m_targetPose = m_targetPose.addTransform(new Translation2d(xSpeed.get() / 10.0, new Rotation2d()));
      m_swerveBase.drive(speeds);
    }

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

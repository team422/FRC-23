
package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveToCubeAuton extends CommandBase {
  Drive m_swerveBase;
  Intake m_intake;
  ExtendedPathPoint m_targetPose;
  Supplier<ExtendedPathPoint> m_startPose;
  Supplier<Pose2d> m_adjustedPoseSupplier;
  double m_maxSpeedWanted;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  ExtendedPathPoint m_basePose;

  public DriveToCubeAuton(Drive swerveBase, Supplier<Pose2d> adjustedPoseSupplier,
      CustomHolmonomicDrive HolmDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation, Intake intake, Supplier<ExtendedPathPoint> basicPose) {
    m_swerveBase = swerveBase;
    m_intake = intake;
    m_startPose = basicPose;
    m_adjustedPoseSupplier = adjustedPoseSupplier;

    m_HolmDrive = HolmDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(swerveBase);

  }

  @Override
  public void initialize() {
    m_basePose = m_startPose.get();
    m_targetPose = m_startPose.get();
  }

  @Override
  public void execute() {
    Pose2d targetPose = m_adjustedPoseSupplier.get();

    if (targetPose != null) {
      double cubeDistanceFromExpectedAuton = targetPose.getTranslation().getDistance(m_basePose.getTranslation());
      Logger.getInstance().recordOutput("Cube/cubeErrorAuton", cubeDistanceFromExpectedAuton);
      if (cubeDistanceFromExpectedAuton < Units.inchesToMeters(24)) {
        m_targetPose = new ExtendedPathPoint(targetPose.getTranslation(), targetPose.getRotation(),
            targetPose.getRotation());
      }
    }
    // System.out.println(m_targetPose.getTranslation().getDistance(m_basePose.getTranslation()));
    ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_targetPose.getPose2d());
    // m_targetPose = m_targetPose.addTransform(new Translation2d(xSpeed.get() / 10.0, new Rotation2d()));
    m_swerveBase.drive(speeds);

  }

  @Override
  public boolean isFinished() {
    // return m_HolmDrive.atReference();
    return m_intake.hasGamePiece() || m_targetPose.atXY(m_swerveBase.getPose());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }
    m_swerveBase.drive(new ChassisSpeeds());
  }

}

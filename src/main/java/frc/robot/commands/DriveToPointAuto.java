package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveToPointAuto extends CommandBase {
  FullSwerveBase m_swerveBase;
  Pose2d m_targetPose;
  double m_maxSpeedWanted;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;

  public DriveToPointAuto(FullSwerveBase swerveBase, Pose2d targetPose, CustomHolmonomicDrive HolmDrive) {
    m_swerveBase = swerveBase;
    m_targetPose = targetPose;
    m_HolmDrive = HolmDrive;

    addRequirements(swerveBase);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_targetPose, () -> {
      return 0.0;
    }, () -> {
      return 0.0;
    }, () -> {
      return 0.0;
    });
    m_swerveBase.drive(speeds);

  }

  @Override
  public boolean isFinished() {
    boolean slow = true;
    for (double velo : m_swerveBase.getVelocities()) {
      if (velo > 0.15) {
        slow = false;
      }
    }
    return m_HolmDrive.atReference() && slow;
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }
    m_swerveBase.drive(new ChassisSpeeds());
  }

}

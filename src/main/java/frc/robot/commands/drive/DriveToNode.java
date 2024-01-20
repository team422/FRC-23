
package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.lib.utils.FieldGeomUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveToNode extends CommandBase {
  Drive m_swerveBase;
  ExtendedPathPoint m_targetPose;
  double m_maxSpeedWanted;
  FieldGeomUtil m_targetPoseSupplier;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;

  public DriveToNode(Drive swerveBase, FieldGeomUtil targetPose,
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
    m_targetPose = m_targetPoseSupplier.getClosestNode(m_swerveBase.getPose());

  }

  @Override
  public void execute() {
    System.out.println(m_targetPose.getPose2d().getRotation().getSin());
    if (Math.round(m_targetPose.getPose2d().getRotation().getSin()) == 0.0) {

      ChassisSpeeds speeds = m_HolmDrive.calculateOnlyY(m_swerveBase.getPose(), m_targetPose.getPose2d(), xSpeed,
          ySpeed,
          zRotation);
      m_swerveBase.drive(speeds);
    } else {
      ChassisSpeeds speeds = m_HolmDrive.calculateOnlyX(m_swerveBase.getPose(), m_targetPose.getPose2d(), xSpeed,
          ySpeed,
          zRotation);
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

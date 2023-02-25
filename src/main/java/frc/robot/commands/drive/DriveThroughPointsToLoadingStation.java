package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.lib.utils.FieldGeomUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolmonomicDrive;

public class DriveThroughPointsToLoadingStation extends CommandBase {
  Drive m_swerveBase;
  Pose2d m_targetPose;
  double m_maxSpeedWanted;
  CustomHolmonomicDrive m_HolmDrive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  ArrayList<ExtendedPathPoint> m_pathPoints;
  ExtendedPathPoint m_desiredPoint;
  FieldGeomUtil m_fieldGeomUtil = new FieldGeomUtil();
  int curPoint;
  Boolean m_finalPoint;

  public DriveThroughPointsToLoadingStation(Drive swerveBase,
      CustomHolmonomicDrive HolmDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation) {
    m_swerveBase = swerveBase;
    m_HolmDrive = HolmDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    addRequirements(swerveBase);

  }

  @Override
  public void initialize() {
    m_pathPoints = m_fieldGeomUtil.fastestPathToLoadingStation(m_swerveBase.getPose(), DriverStation.getAlliance(), 1);
    m_desiredPoint = m_pathPoints.get(0);
    m_HolmDrive.setTolerance(m_desiredPoint.getTolerance());
    curPoint = 0;
    m_finalPoint = false;
  }

  @Override
  public void execute() {

    ChassisSpeeds speeds = m_HolmDrive.calculate(m_swerveBase.getPose(), m_desiredPoint, xSpeed, ySpeed, zRotation,
        m_finalPoint);
    m_swerveBase.drive(speeds);
    if (m_HolmDrive.atReference()) {
      if (curPoint == m_pathPoints.size() - 1) {
        m_finalPoint = true;

      } else {
        curPoint += 1;
        m_HolmDrive.setTolerance(m_pathPoints.get(curPoint).getTolerance());
        m_desiredPoint = m_pathPoints.get(curPoint);
      }
    }

  }

  @Override
  public boolean isFinished() {
    // System.out.println((curPoint == m_pathPoints.size() - 1) && m_HolmDrive.atReference());
    return (m_finalPoint) && m_HolmDrive.atReference();
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

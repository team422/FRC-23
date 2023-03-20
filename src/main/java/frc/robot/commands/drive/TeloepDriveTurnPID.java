package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomHolmonomicDrive;
import frc.robot.util.EricNubControls;

public class TeloepDriveTurnPID extends CommandBase {
  Drive m_drive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  double curXSpeed;
  double curYSpeed;
  double curZSpeed;
  double curXLocation;
  double curYLocation;
  double curZRotation;
  double deadzone;
  double m_acceptedXYError;
  Rotation2d m_acceptedZError;
  ChassisSpeeds speeds;
  EricNubControls m_controlsHandler;

  CustomHolmonomicDrive m_customDriveControls;

  boolean wasTurning = false;
  boolean wasMovingX = false;
  boolean wasMovingY = false;

  public TeloepDriveTurnPID(Drive drive, CustomHolmonomicDrive customDriveControls,
      Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation, double deadzone, double acceptedXYError, Rotation2d acceptedZError) {
    m_drive = drive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.deadzone = deadzone;
    m_customDriveControls = customDriveControls;
    m_acceptedXYError = acceptedXYError;
    m_acceptedZError = acceptedZError;
    m_controlsHandler = new EricNubControls();
    Pose2d pose = m_drive.getPose();
    curXLocation = pose.getTranslation().getX();
    curYLocation = pose.getTranslation().getY();
    curZRotation = pose.getRotation().getRadians();

    addRequirements(drive);

  }

  public void execute() {
    curXSpeed = xSpeed.get();
    curYSpeed = ySpeed.get();
    curZSpeed = zRotation.get();
    Pose2d pose = m_drive.getPose();
    // curXSpeed = MathUtil.applyDeadband(curZRotation, curYSpeed)
    curXSpeed = m_controlsHandler.addDeadzoneScaled(curXSpeed, deadzone);
    curYSpeed = m_controlsHandler.addDeadzoneScaled(curYSpeed, deadzone);
    curZSpeed = m_controlsHandler.addDeadzoneScaled(curZSpeed, deadzone);

    curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curZSpeed *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    // use slew rate limiter

    // System.out.println(curXSpeed);
    // if the robot stops turning and was turning, reset the location
    if (curZSpeed == 0 && wasTurning) {
      curZRotation = pose.getRotation().getRadians();
      wasTurning = false;
    } else if (curZSpeed != 0) {
      wasTurning = true;
    }
    if (curXSpeed == 0 && wasMovingX) {
      curXLocation = pose.getX();
      wasMovingX = false;
    } else if (curXSpeed != 0) {
      wasMovingX = true;
    }
    if (curYSpeed == 0 && wasMovingY) {
      curYLocation = pose.getY();
      wasMovingY = false;
    } else if (curYSpeed != 0) {
      wasMovingY = true;
    }
    // curXSpeed = m_customDriveControls.slewRateX(curXSpeed);
    // curYSpeed = m_customDriveControls.slewRateY(curYSpeed);
    // curZSpeed = m_customDriveControls.slewRateZ(curZSpeed);

    curXLocation += curXSpeed * 0.02;
    curYLocation += curYSpeed * 0.02;
    curZRotation += curZSpeed * 0.02;

    Pose2d desPose = new Pose2d(curXLocation, curYLocation, new Rotation2d(curZRotation));
    m_customDriveControls.distanceFromXY(pose, desPose);
    m_customDriveControls.distanceFromZ(pose, desPose);
    // if (m_customDriveControls.distanceFromXY(pose, desPose) > m_acceptedXYError) {
    //   curXLocation = pose.getX() + curXSpeed * 0.02;
    //   curYLocation = pose.getY() + curYSpeed * 0.02;
    // }
    // if (m_customDriveControls.distanceFromZ(pose, desPose).minus(m_acceptedZError)
    //     .getDegrees() > 0) {
    //   curZRotation = pose.getRotation().getRadians() + curZSpeed;
    // }

    ChassisSpeeds speeds = m_customDriveControls.fullCalulate(desPose,
        pose, m_drive.getChassisSpeeds());
    Logger.getInstance().recordOutput("Drive/desPose", desPose);
    m_drive.drive(speeds);

  }

}

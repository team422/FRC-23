package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
  Pose2d errPose2d;
  double deadzone;
  double m_acceptedXYError;
  Rotation2d m_acceptedZError;
  ChassisSpeeds speeds;
  EricNubControls m_controlsHandler;
  Pose2d oldPose;

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
    oldPose = m_drive.getPose();
    errPose2d = new Pose2d(0, 0, new Rotation2d(0));

    addRequirements(drive);

  }

  public void execute() {
    curXSpeed = xSpeed.get();
    curYSpeed = ySpeed.get();
    curZSpeed = zRotation.get();
    Pose2d pose = m_drive.getPose();

    Translation2d translation = oldPose.getTranslation().minus(pose.getTranslation());
    Translation2d errTranslation2d = errPose2d.getTranslation().plus(translation);
    Rotation2d errRotation2d = errPose2d.getRotation().minus(pose.getRotation().minus(oldPose.getRotation()));
    if (m_drive.hasResetOdometry()) {
      errTranslation2d = new Translation2d();
      errRotation2d = new Rotation2d();
    }
    errPose2d = new Pose2d(errTranslation2d, errRotation2d);

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
    ChassisSpeeds curChassisSpeeds = m_drive.getChassisSpeeds();

    if (curZSpeed == 0 && wasTurning) {
      errPose2d = new Pose2d(errPose2d.getX(), errPose2d.getY(),
          Rotation2d.fromRadians(curChassisSpeeds.omegaRadiansPerSecond).times(0.05));
      wasTurning = false;
    } else {
      wasTurning = true;
    }
    if (curXSpeed == 0) {
      errPose2d = new Pose2d(0, errPose2d.getY(), errPose2d.getRotation());
    }
    if (curYSpeed == 0) {
      errPose2d = new Pose2d(errPose2d.getX(), 0, errPose2d.getRotation());
    }
    //   wasMovingX = true;
    // }
    // if (curYSpeed == 0 && wasMovingY) {
    //   errYLocation = pose.getY();
    //   wasMovingY = false;
    // } else if (curYSpeed != 0) {
    //   wasMovingY = true;
    // }
    // curXSpeed = m_customDriveControls.slewRateX(curXSpeed);
    // curYSpeed = m_customDriveControls.slewRateY(curYSpeed);
    // curZSpeed = m_customDriveControls.slewRateZ(curZSpeed);
    // Pose2d deltaErr = new Pose2d(curXSpeed * 0.02, curYSpeed * 0.02,
    //     new Rotation2d(curZSpeed * 0.02));
    // Transform2d deltaErr = new Transform2d(new Translation2d(curXSpeed * 0.02, curYSpeed * 0.02),
    //     new Rotation2d(curZSpeed * 0.02));
    Translation2d deltaErrTranslation2d = new Translation2d(curXSpeed * 0.02, curYSpeed * 0.02);
    Rotation2d deltaErrRotation2d = new Rotation2d(curZSpeed * 0.02);
    errPose2d = new Pose2d(errPose2d.getTranslation().plus(deltaErrTranslation2d),
        errPose2d.getRotation().plus(deltaErrRotation2d));

    // errPose2d = errPose2d.plus(deltaErr);
    // Pose2d desPose = new Pose2d(errXLocation, errYLocation, new Rotation2d(errZRotation));
    Pose2d desPose = new Pose2d(pose.getX() + errPose2d.getX(), pose.getY() + errPose2d.getY(),
        pose.getRotation().plus(errPose2d.getRotation()));
    m_customDriveControls.distanceFromXY(pose, desPose);
    m_customDriveControls.distanceFromZ(pose, desPose);
    oldPose = pose;
    if (m_customDriveControls.distanceFromXY(errPose2d, new Pose2d()) > 3) {
      // deflate the error to 1 m
      errPose2d = errPose2d.times(1 / m_customDriveControls.distanceFromXY(errPose2d, new Pose2d()));

    }
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
    speeds.omegaRadiansPerSecond = m_controlsHandler.addDeadzoneScaled(speeds.omegaRadiansPerSecond,
        Units.degreesToRadians(1));
    speeds.vxMetersPerSecond = m_controlsHandler.addDeadzoneScaled(speeds.vxMetersPerSecond,
        Units.inchesToMeters(1));
    speeds.vyMetersPerSecond = m_controlsHandler.addDeadzoneScaled(speeds.vyMetersPerSecond,
        Units.inchesToMeters(1));
    m_drive.drive(speeds);

  }

}

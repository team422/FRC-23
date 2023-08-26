package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.EricNubControls;

public class TeloepDrive extends CommandBase {
  Drive m_drive;
  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  double curXSpeed;
  double curYSpeed;
  double curZRotation;
  double deadzone;
  ChassisSpeeds speeds;
  EricNubControls m_controlsHandler;

  public TeloepDrive(Drive drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation, double deadzone) {
    m_drive = drive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.deadzone = deadzone;
    m_controlsHandler = new EricNubControls();
    addRequirements(drive);

  }

  public void execute() {
    curXSpeed = xSpeed.get();
    curYSpeed = ySpeed.get();
    curZRotation = zRotation.get();
    // curXSpeed = MathUtil.applyDeadband(curZRotation, curYSpeed)
    curXSpeed = m_controlsHandler.addDeadzoneScaled(curXSpeed, deadzone);
    curYSpeed = m_controlsHandler.addDeadzoneScaled(curYSpeed, deadzone);
    curZRotation = m_controlsHandler.addDeadzoneScaled(curZRotation, deadzone);

    curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curZRotation *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    // System.out.println(curXSpeed);

    //calculate the speeds from applying calculated Twist2d to robotPoseVel
    Pose2d robotPoseVel = new Pose2d(curXSpeed * 0.02, curYSpeed * 0.02, Rotation2d.fromRadians(curZRotation * 0.02));
    Twist2d twistVel = new Pose2d(0, 0, Rotation2d.fromRadians(0)).log(robotPoseVel);
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(twistVel.dx / 0.02, twistVel.dy / 0.02, twistVel.dtheta / 0.02,
        m_drive.getPose().getRotation());

    // speeds = new ChassisSpeeds(curXSpeed, curYSpeed, curZRotation);
    m_drive.drive(speeds);
  }

}

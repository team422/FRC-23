package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.util.EricNubControls;

public class FullSwerveDrive extends CommandBase {

  Supplier<Double> xSpeed;
  Supplier<Double> ySpeed;
  Supplier<Double> zRotation;
  double curXSpeed;
  double curYSpeed;
  double curZRotation;
  FullSwerveBase swerveBase;
  ChassisSpeeds speeds;
  EricNubControls controlsHandler = new EricNubControls();

  public FullSwerveDrive(FullSwerveBase swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
      Supplier<Double> zRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
    this.swerveBase = swerveDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
  }

  public void initialize() {
    // swerveBase.drive(xSpeed.get(), ySpeed.get(), zRotation.get());
  }

  public void execute() {
    curXSpeed = controlsHandler.addDeadzoneScaled(xSpeed.get(), .1);
    curYSpeed = controlsHandler.addDeadzoneScaled(ySpeed.get(), 0.1);
    curZRotation = controlsHandler.addDeadzoneScaled(zRotation.get(), 0.1);
    // curXSpeed = controlsHandler.addEricCurve(curXSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    // curYSpeed = controlsHandler.addEricCurve(curYSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    // curZRotation = controlsHandler.addEricCurve(curZRotation) * DriveConstants.kMaxAngularSpeed;
    if (RobotState.isTeleop()) {
      curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curZRotation *= DriveConstants.kMaxAngularSpeed;
    } else if (RobotState.isAutonomous()) {
      curXSpeed *= DriveConstants.kMaxAutoSpeedMetersPerSecond;
      curYSpeed *= DriveConstants.kMaxAutoSpeedMetersPerSecond;
      curZRotation *= DriveConstants.KMaxAutoRotationPerSecond;
    } else {
      curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curZRotation *= DriveConstants.kMaxAngularSpeed;
    }

    // if (curXSpeed != 0 || curYSpeed != 0 || curZRotation != 0) {
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        swerveBase.getPose().getRotation());
    swerveBase.drive(speeds);
    // }
    // speeds = new ChassisSpeeds(xSpeed.get(), ySpeed.get(),
    //         zRotation.get() * DriveConstants.kMaxRotationalVelocity);
    // swerveBase.drive(speeds);
    // }
    // swerveBase.drive(xSpeed.get(), ySpeed.get(), zRotation.get());
  }

}

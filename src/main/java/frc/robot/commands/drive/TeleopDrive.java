package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.EricNubControls;

public class TeleopDrive extends CommandBase {
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

  public TeleopDrive(Drive drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed,
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
    if (RobotState.getInstance().elevatorYMeters < Units.inchesToMeters(20)) {

      curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
      curZRotation *= DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    } else {
      // curXSpeed = MathUtil.applyDeadband(curZRotation, curYSpeed)
      curXSpeed = Math.signum(curXSpeed) * Math.sqrt(Math.abs(curXSpeed));
      curYSpeed = Math.signum(curYSpeed) * Math.sqrt(Math.abs(curYSpeed));
      curZRotation = Math.signum(curZRotation) * Math.sqrt(Math.abs(curZRotation));

      curXSpeed = Math.signum(curXSpeed) * Math.sqrt(Math.abs(curXSpeed));
      curYSpeed = Math.signum(curYSpeed) * Math.sqrt(Math.abs(curYSpeed));
      curZRotation = Math.signum(curZRotation) * Math.sqrt(Math.abs(curZRotation));

      curXSpeed *= DriveConstants.kMaxHighElevatorSpeedMetersPerSecond;
      curYSpeed *= DriveConstants.kMaxHighElevatorSpeedMetersPerSecond;
      curZRotation *= DriveConstants.kMaxHighElevatorAngularSpeedRadiansPerSecond;
    }
    // System.out.println(curXSpeed);

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        m_drive.getPose().getRotation());
    // speeds = new ChassisSpeeds(curXSpeed, curYSpeed, curZRotation);
    m_drive.drive(speeds);
  }

}

package frc.robot.commands.drive;

import java.util.function.Supplier;

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

    curXSpeed = m_controlsHandler.addDeadzoneScaled(curXSpeed, deadzone);
    curYSpeed = m_controlsHandler.addDeadzoneScaled(curYSpeed, deadzone);
    curZRotation = m_controlsHandler.addDeadzoneScaled(curZRotation, deadzone);

    curXSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curYSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    curZRotation *= DriveConstants.kMaxSpeedMetersPerSecond;
    // System.out.println(curXSpeed);

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(curXSpeed, curYSpeed, curZRotation,
        m_drive.getPose().getRotation());
    m_drive.drive(speeds);
  }

}

package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;

public class ChargeStationBalance extends CommandBase {
  private static final double kMaxIncline = Units.degreesToRadians(10);
  private static final double kLevelThreshold = Units.degreesToRadians(2);
  private static final double kAngleSpeedMultiplier = 0.7;
  private static final double kMaxBalanceSpeed = 1.0;
  private static final double kMountSpeed = 4.0;
  private final Drive m_drive;
  private final GyroIO m_gyro;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    m_gyro = drive.getGyro();
  }

  @Override
  public void initialize() {
    m_drive.setDriveBrakeMode(true);
    m_drive.setTurnBrakeMode(true);
  }

  @Override
  public void execute() {
    var upVector = m_gyro.getUpVector();

    // This gets the angle of the robot in the XZ plane
    Rotation2d angle = new Rotation2d(upVector.get(2, 0), upVector.get(0, 0));

    // Apply a deadband to the angle. Essentially telling the robot to stop moving as we approach level
    double deadbandAngle = MathUtil.applyDeadband(angle.getRadians(), kLevelThreshold, kMaxIncline);

    // Calculate the speed based on the angle (bigger angle = bigger speed)
    double xSpeed = deadbandAngle * kAngleSpeedMultiplier;

    // Prevent the calculated speed from exceeding the max speed
    xSpeed = MathUtil.clamp(xSpeed, -kMaxBalanceSpeed, kMaxBalanceSpeed);

    var outputSpeeds = new ChassisSpeeds(xSpeed, 0, 0);

    m_drive.drive(outputSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished!" + interrupted);
    m_drive.brake();
  }

  /**
   * Mount onto the charging station, then try to level out
   * @param drive
   * @return
   */
  public static CommandBase charge(Drive drive) {
    return Commands.sequence(
        mountChargingStation(drive),
        new ChargeStationBalance(drive));
  }

  /**
   * Drive forward until the drivebase is no longer level, or after 1.5 seconds
   * @param drive
   * @return
   */
  private static CommandBase mountChargingStation(Drive drive) {
    return drive
        .driveCommand(() -> {
          // Calculate where the robot is in relation to the charge station
          double robotOffsetX = FieldConstants.kChargeStationCenter.getX() - drive.getPose().getX();

          // Set the speed in that direction
          double speed = Math.copySign(kMountSpeed, robotOffsetX);
          return ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, drive.getPose().getRotation());
        })
        .until(() -> !drive.getGyro().isLevel(kMaxIncline))
        .withTimeout(1.5);
  }
}

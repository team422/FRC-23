package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;

public class ChargeStationBalance extends CommandBase {
  /**
   *
   */
  private static final double kMaxIncline = Units.degreesToRadians(10);
  private static final double kLevelThreshold = Units.degreesToRadians(2);
  private static final double kAngleSpeedMultiplier = 0.7;
  private static final double kMaxSpeed = 2.0;
  private static final double kRoot2Over2 = Math.sqrt(2) / 2;
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
    // TODO: utilize both pitch and roll
    // boolean usePitch = Math.abs(gyro.getRotation2d().getCos()) < root2Over2;
    // double angle = usePitch ? gyro.getPitch() : gyro.getRoll();

    double angle = -m_gyro.getPitch().getRadians();
    double deadbandAngle = MathUtil.applyDeadband(angle, kLevelThreshold, kMaxIncline);
    double xSpeed = deadbandAngle * kAngleSpeedMultiplier;

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
        .forwardCommand(0.5)
        .until(() -> !drive.getGyro().isLevel(kMaxIncline))
        .withTimeout(1.5);
  }
}

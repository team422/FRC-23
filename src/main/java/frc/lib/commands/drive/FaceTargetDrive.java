package frc.lib.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class FaceTargetDrive extends TargetAngleDrive {
  private static final double kPredictiveMultiplier = 5.5;
  private final Supplier<Translation2d> m_targetSupplier;

  public FaceTargetDrive(DriveCommandConfig drive, ProfiledPIDController controller,
      Supplier<Translation2d> targetSupplier) {
    super(drive, controller);
    m_targetSupplier = targetSupplier;
  }

  public FaceTargetDrive(DriveCommandConfig drive, ProfiledPIDController controller, Translation2d target) {
    this(drive, controller, () -> target);
  }

  public FaceTargetDrive(DriveCommandConfig drive, ProfiledPIDController controller, Pose2d target) {
    this(drive, controller, () -> target.getTranslation());
  }

  @Override
  protected Rotation2d getDesiredAngle() {
    // Pose2d robotPose = getPredictiveRobotPose();
    Pose2d robotPose = m_drive.getPose();
    Translation2d target = m_targetSupplier.get();

    Translation2d offset = target.minus(robotPose.getTranslation());

    return new Rotation2d(offset.getX(), offset.getY());
  }

  protected Pose2d getPredictiveRobotPose() {
    var chassisSpeeds = m_drive.getCurrentSpeeds();
    double deltaX = chassisSpeeds.vxMetersPerSecond * 0.02 * kPredictiveMultiplier;
    double deltaY = chassisSpeeds.vyMetersPerSecond * 0.02 * kPredictiveMultiplier;
    double deltaTheta = chassisSpeeds.omegaRadiansPerSecond * 0.02 * kPredictiveMultiplier;

    return m_drive.getPose().exp(new Twist2d(deltaX, deltaY, deltaTheta));
  }
}

package frc.lib.commands.drive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Describes a generic swerve drive subsystem
 */
public class DriveCommandConfig {
  private final SwerveDriveKinematics m_kinematics;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Supplier<SwerveModuleState[]> m_stateSupplier;
  private final Supplier<Double> m_gyroRateRadsPerSecondSupplier;
  private final Consumer<ChassisSpeeds> m_speedsConsumer;
  private final Subsystem[] m_requirements;

  public DriveCommandConfig(
      SwerveDriveKinematics kinematics,
      Supplier<Pose2d> poseSupplier,
      Supplier<SwerveModuleState[]> stateSupplier,
      Supplier<Double> gyroRateRadsPerSecondSupplier,
      Consumer<ChassisSpeeds> speedsConsumer,
      Subsystem... requirements) {
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "DriveCommandConfig");
    m_poseSupplier = requireNonNullParam(poseSupplier, "poseSupplier", "DriveCommandConfig");
    m_speedsConsumer = requireNonNullParam(speedsConsumer, "speedsConsumer", "DriveCommandConfig");
    m_stateSupplier = requireNonNullParam(stateSupplier, "speedsSupplier", "DriveCommandConfig");
    m_gyroRateRadsPerSecondSupplier = requireNonNullParam(gyroRateRadsPerSecondSupplier,
        "gyroRateRadsPerSecondSupplier",
        "DriveCommandConfig");
    m_requirements = requirements;
  }

  public Pose2d getPose() {
    return m_poseSupplier.get();
  }

  public double getGyroRate() {
    return m_gyroRateRadsPerSecondSupplier.get();
  }

  public SwerveModuleState[] getStates() {
    return m_stateSupplier.get();
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return m_kinematics.toChassisSpeeds(getStates());
  }

  public void acceptSpeeds(ChassisSpeeds speeds) {
    m_speedsConsumer.accept(speeds);
  }

  public Subsystem[] getRequirements() {
    return this.m_requirements;
  }
}

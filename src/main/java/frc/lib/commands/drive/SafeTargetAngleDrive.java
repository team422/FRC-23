package frc.lib.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class SafeTargetAngleDrive extends TargetAngleDrive {
  private Rotation2d m_previousAngle;

  public SafeTargetAngleDrive(DriveCommandConfig drive, ProfiledPIDController controller) {
    super(drive, controller);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_previousAngle = m_drive.getPose().getRotation();
  }

  @Override
  protected final Rotation2d getDesiredAngle() {
    m_previousAngle = getDesiredSafeAngle().orElse(m_previousAngle);
    return m_previousAngle;
  }

  protected abstract Optional<Rotation2d> getDesiredSafeAngle();
}

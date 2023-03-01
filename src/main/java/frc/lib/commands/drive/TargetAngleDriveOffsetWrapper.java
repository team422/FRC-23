package frc.lib.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class TargetAngleDriveOffsetWrapper extends TargetAngleDriveWrapper {
  private final Supplier<Rotation2d> m_offset;

  public TargetAngleDriveOffsetWrapper(TargetAngleDrive targetAngleDrive, Rotation2d offset) {
    this(targetAngleDrive, () -> offset);
  }

  public TargetAngleDriveOffsetWrapper(TargetAngleDrive targetAngleDrive, Supplier<Rotation2d> offset) {
    super(targetAngleDrive);
    m_offset = offset;
  }

  @Override
  protected Rotation2d getDesiredAngle() {
    return super.getDesiredAngle().plus(m_offset.get());
  }
}

package frc.lib.commands.drive.turn;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.commands.drive.DriveCommandConfig;

public class TurnToFaceTarget extends BaseTurnCommand {

  private Supplier<Translation2d> m_targetSupplier;

  public TurnToFaceTarget(DriveCommandConfig drive, ProfiledPIDController controller,
      Supplier<Translation2d> targetSupplier) {
    super(drive, controller);
    m_targetSupplier = targetSupplier;
  }

  public TurnToFaceTarget(DriveCommandConfig drive, ProfiledPIDController controller, Translation2d target) {
    this(drive, controller, () -> target);
  }

  public TurnToFaceTarget(DriveCommandConfig drive, ProfiledPIDController controller, Pose2d target) {
    this(drive, controller, target.getTranslation());
  }

  @Override
  public Rotation2d getDesiredAngle() {
    Translation2d offset = m_targetSupplier.get().minus(m_drive.getPose().getTranslation());
    System.out.println(offset);
    return new Rotation2d(offset.getX(), offset.getY());
  }

}

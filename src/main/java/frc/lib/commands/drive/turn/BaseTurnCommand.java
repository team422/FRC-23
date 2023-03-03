package frc.lib.commands.drive.turn;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.lib.commands.drive.TargetAngleDrive;

public abstract class BaseTurnCommand extends TargetAngleDrive {

  public BaseTurnCommand(DriveCommandConfig drive, ProfiledPIDController controller) {
    super(drive, controller);
  }

  @Override
  protected final double getXSpeed() {
    return 0;
  }

  @Override
  protected final double getYSpeed() {
    return 0;
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}

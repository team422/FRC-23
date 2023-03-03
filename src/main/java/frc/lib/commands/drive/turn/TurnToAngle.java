package frc.lib.commands.drive.turn;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.commands.drive.DriveCommandConfig;

public class TurnToAngle extends BaseTurnCommand {
  protected final Rotation2d m_desiredRobotAngle;

  public TurnToAngle(DriveCommandConfig drive, ProfiledPIDController controller, Rotation2d angle) {
    super(drive, controller);
    m_desiredRobotAngle = angle;
  }

  /**
   * @return The desired angle in radians
   */
  public Rotation2d getDesiredAngle() {
    return m_desiredRobotAngle;
  }

}

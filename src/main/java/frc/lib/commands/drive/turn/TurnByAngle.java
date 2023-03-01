package frc.lib.commands.drive.turn;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.commands.drive.DriveCommandConfig;

public class TurnByAngle extends BaseTurnCommand {
  private Rotation2d m_inputTurnAngle;
  private Rotation2d m_desiredRobotAngle;

  public TurnByAngle(DriveCommandConfig drive, ProfiledPIDController controller, Rotation2d angle) {
    super(drive, controller);
    m_inputTurnAngle = angle;
  }

  @Override
  public void initialize() {
    m_desiredRobotAngle = m_drive.getPose().getRotation().plus(m_inputTurnAngle);
  }

  @Override
  public Rotation2d getDesiredAngle() {
    return m_desiredRobotAngle;
  }

}

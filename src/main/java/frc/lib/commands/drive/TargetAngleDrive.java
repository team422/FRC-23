package frc.lib.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class TargetAngleDrive extends DriveCommandAdapter {
  protected final ProfiledPIDController m_controller;

  public TargetAngleDrive(DriveCommandConfig config, ProfiledPIDController controller) {
    super(config);

    m_controller = controller;

    m_controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_controller.reset(
        m_drive.getPose().getRotation().getRadians(),
        m_drive.getGyroRate());
  }

  @Override
  protected final double getRotationSpeed() {
    Rotation2d currentRotation = m_drive.getPose().getRotation();
    Rotation2d desiredRotation = getDesiredAngle();

    return m_controller.calculate(currentRotation.getRadians(), desiredRotation.getRadians());
  }

  /**
   * 
   * @param offset
   * @return a new instance of the TargetAngleDrive with a specified offset rotation
   */
  public final TargetAngleDrive withAngleOffset(Rotation2d offset) {
    return new TargetAngleDriveOffsetWrapper(this, offset);
  }

  /**
   * To use FaceTargetDrive as an example, where the robot will face towards a given target,
   * this method will return an instance that faces directly opposite from the given target.
   * @return a new instance of the TargetAngleDrive with an offset of 180 degrees
   */
  public final TargetAngleDrive facingAway() {
    return withAngleOffset(Rotation2d.fromDegrees(180));
  }

  /**
   * 
   * @return the target robot gyro angle
   */
  protected abstract Rotation2d getDesiredAngle();
}

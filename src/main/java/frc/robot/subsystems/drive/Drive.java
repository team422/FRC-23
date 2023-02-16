package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.gyro.GyroSub;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;
  private Pose2d curPose = new Pose2d();
  private SwerveDrivePoseEstimator m_odometry;
  private GyroSub m_gyro;

  /** Creates a new Drive. */
  public Drive(GyroSub gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleInputsAutoLogged();
    }
    m_odometry = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose

    );
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getModulePosition();
    }
    return positions;
  }

  public void drive(ChassisSpeeds speeds) {

    // Set chassis speeds

  }

  public Pose2d getPose() {
    // Return the current pose
    return new Pose2d();
  }

  public void brake() {
    // Set chassis speeds to 0
  }

  public CommandBase brakeCommand() {
    return runOnce(this::brake);
  }
}

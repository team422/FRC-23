package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.gyro.GyroSub;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;
  private Pose2d curPose = new Pose2d();
  private SwerveDrivePoseEstimator m_odometry;
  private SwerveModuleIO[] m_swerveModules;
  private GyroSub m_gyro;

  /** Creates a new Drive. */
  public Drive(GyroSub gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_swerveModules = modules;
    for (SwerveModuleIO module : m_modules) {
      module.resetDistance();
      module.syncTurningEncoder();
    }
    m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleInputsAutoLogged();
    }
    m_odometry = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose);
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
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveModuleState[] moduleStatesFinal = new SwerveModuleState[4];
    if (speeds.omegaRadiansPerSecond == 0 && speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0) {
      this.brake();
    } else {
      for (int i = 0; i < 4; i++) {
        moduleStatesFinal[i] = SwerveModuleState.optimize(moduleStates[i], m_swerveModules[i].getTurnDegrees());
      }
      setModuleStates(moduleStatesFinal);
    }
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < 4; i++) {
      m_swerveModules[i].setDesiredState(moduleStates[i]);
    }
  }

  public Pose2d getPose() {
    // Return the current pose
    return m_odometry.getEstimatedPosition();
  }

  public void brake() {
    // Set chassis speeds to 0
    this.drive(new ChassisSpeeds(0, 0, 0));
  }

  public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
  }

  public CommandBase brakeCommand() {
    return runOnce(this::brake);
  }
}

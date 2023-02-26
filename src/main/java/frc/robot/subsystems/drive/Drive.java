package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;
  private Pose2d curPose = new Pose2d();
  private SwerveDrivePoseEstimator m_odometry;
  private SwerveModuleIO[] m_swerveModules;
  private GyroIO m_gyro;
  private GyroInputsAutoLogged m_gyroInputs;
  private double m_simGyroLastUpdated;

  /** Creates a new Drive. */
  public Drive(GyroIO gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();
    m_swerveModules = modules;
    for (SwerveModuleIO module : m_modules) {
      module.resetDistance();
      module.syncTurningEncoder();
      // module.resetEncoders();
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
    m_gyro.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("Gyro", m_gyroInputs);
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
      Logger.getInstance().recordOutput("Module" + i, m_modules[i].getSwerveModuleState());

    }
    m_odometry.update(m_gyro.getAngle(), getSwerveModulePositions());
    Logger.getInstance().recordOutput("Drive/OdometryX", m_odometry.getEstimatedPosition().getX());
    Logger.getInstance().recordOutput("Drive/OdometryY", m_odometry.getEstimatedPosition().getY());
    Logger.getInstance().recordOutput("Drive/OdometryZ", m_odometry.getEstimatedPosition().getRotation().getDegrees());
    Logger.getInstance().recordOutput("Drive/Pose", m_odometry.getEstimatedPosition());
    FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getSwerveModuleStates(),
        DriveConstants.kModuleTranslations);

  }

  @Override
  public void simulationPeriodic() {
    double gyroDelta = getChassisSpeeds().omegaRadiansPerSecond;
    double ts = Timer.getFPGATimestamp();

    double deltaTime = ts - m_simGyroLastUpdated;

    m_gyro.addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
    m_simGyroLastUpdated = ts;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void resetOdometry() {
    m_odometry.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getSwerveModuleState();
    }
    return positions;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], m_swerveModules[i].getAngle());
    }

    double angularMultiplier = RobotState.getInstance().getMorphedVelocityMultiplier();
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates,
        speeds,
        DriveConstants.kMaxModuleSpeedMetersPerSecond,
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAngularSpeedRadiansPerSecond * angularMultiplier);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < moduleStates.length; i++) {
      m_swerveModules[i].setDesiredState(moduleStates[i]);
    }
  }

  public Pose2d getPose() {
    // Return the current pose
    return m_odometry.getEstimatedPosition();
  }

  public void brake() {
    // Set chassis speeds to 0
    // setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
  }

  public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
  }

  public GyroIO getGyro() {
    return m_gyro;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_odometry;
  }

  public Command brakeCommand() {
    return runOnce(this::brake);
  }

  public Command resetCommand() {
    return runOnce(this::resetOdometry);
  }

}

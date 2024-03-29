package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.FieldGeomUtil;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final GyroIO m_gyro;
  private final GyroInputsAutoLogged m_gyroInputs;

  private final double[] m_lockAngles = new double[] { 45, 315, 45, 315 };
  private boolean m_hasResetOdometry;
  private double m_simGyroLastUpdated;
  private double m_lastFPGATimestamp;

  /** Creates a new Drive. */
  public Drive(GyroIO gyro, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();
    for (int i = 0; i < 10; i++) {
      for (SwerveModuleIO module : m_modules) {
        module.resetDistance();
        module.syncTurningEncoder();
        // module.resetEncoders();
      }
    }

    m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleInputsAutoLogged();
    }
    m_poseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose);
    m_hasResetOdometry = false;
    m_lastFPGATimestamp = Timer.getFPGATimestamp();
  }

  public Command resetFirmwareCommand() {
    return Commands.none();
    // return run(() -> {
    //   for (SwerveModuleIO module : m_modules) {
    //     module.syncTurningEncoder();
    //     // module.resetEncoders();
    //   }
    // });
  }

  public void resetFirmware() {
    for (SwerveModuleIO module : m_modules) {
      module.syncTurningEncoder();
      module.setUpModuleFirmware();
    }
  }

  @Override
  public void periodic() {
    m_gyro.updateInputs(m_gyroInputs);
    // Logger.getInstance().processInputs("Gyro", m_gyroInputs);

    double cubeDistanceFromExpectedAuton = new FieldGeomUtil().allGamePieces.get("bumpFar").getTranslation()
        .getDistance(getPose().getTranslation());
    Logger.getInstance().recordOutput("Cube/cubeErrorAutonBumpFar", cubeDistanceFromExpectedAuton);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
    }
    m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());

    Logger.getInstance().recordOutput("Drive/Pose", getPose());
    Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    if (RobotConstants.AScopeLogging) {
      FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
          DriveConstants.kModuleTranslations);
    }
    if (m_lastFPGATimestamp < Timer.getFPGATimestamp()) {
      m_lastFPGATimestamp = Timer.getFPGATimestamp() + 1;
      resetFirmware();
    }

  }

  @Override
  public void simulationPeriodic() {
    double gyroDelta = getChassisSpeeds().omegaRadiansPerSecond;
    double ts = Timer.getFPGATimestamp();
    Logger.getInstance().recordOutput("Drive/Pose", getPose());
    Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    if (RobotConstants.AScopeLogging) {
      FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
          DriveConstants.kModuleTranslations);
    }
    double deltaTime = ts - m_simGyroLastUpdated;

    m_gyro.addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
    m_simGyroLastUpdated = ts;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void resetOdometry() {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(),
        new Pose2d());//1.80, 1.14, new Rotation2d()
    m_hasResetOdometry = true;
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
    m_hasResetOdometry = true;
  }

  public boolean hasResetOdometry() {
    if (m_hasResetOdometry) {
      m_hasResetOdometry = false;
      return true;
    } else {

      return false;
    }
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getState();
    }
    return positions;
  }

  public Command xBrakeCommand() {
    return run(this::xBrake);
  }

  public Command xBrakeInstantCommand() {
    return runOnce(this::xBrake);
  }

  public void xBrake() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_lockAngles[i])));
    }
  }

  public SwerveModuleState[] getModuleAbsoluteStates() {
    SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getAbsoluteState();
    }
    return positions;
  }

  public void drive(ChassisSpeeds speeds) {

    double angularMultiplier = RobotState.getInstance().getMorphedVelocityMultiplier();
    speeds.omegaRadiansPerSecond *= angularMultiplier;

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     moduleStates,
    //     speeds,
    //     DriveConstants.kMaxModuleSpeedMetersPerSecond,
    //     DriveConstants.kMaxSpeedMetersPerSecond,
    //     DriveConstants.kMaxAngularSpeedRadiansPerSecond);

    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], m_modules[i].getAngle());
    }
    Logger.getInstance().recordOutput("Drive/DesiredSpeeds",
        moduleStates);

    // Logger.getInstance().recordOutput("Drive/DesiredModuleStates", moduleStates);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < moduleStates.length; i++) {
      m_modules[i].setDesiredState(moduleStates[i]);
    }
  }

  public Pose2d getPose() {
    // Return the current pose
    return m_poseEstimator.getEstimatedPosition();
  }

  public void brake() {
    // Set chassis speeds to 0
    // setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
  }

  public void addVisionOdometryMeasurement(Pose3d pose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(pose.toPose2d(), timestampSeconds);
  }

  public GyroIO getGyro() {
    return m_gyro;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  public Command brakeCommand() {
    return runOnce(this::brake);
  }

  public Command resetCommand(Pose2d resetPose) {
    return runOnce(() -> {
      resetPose(resetPose);
    });
  }

}

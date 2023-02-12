package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.drive.module.SwerveModuleIO;
import frc.robot.subsystems.drive.module.SwerveModuleInputsAutoLogged;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_moduleInputs;

  private final GyroIO m_gyro;
  private final GyroInputsAutoLogged m_gyroInputs;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new Drive. */
  public Drive(GyroIO gyro, SwerveModuleIO... modules) {
    m_modules = modules;
    m_moduleInputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_moduleInputs.length; i++) {
      m_moduleInputs[i] = new SwerveModuleInputsAutoLogged();
    }

    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();

    for (SwerveModuleIO module : m_modules) {
      module.zeroDriveEncoder();
      module.syncTurnEncoderWithAbsolute();
    }

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroHeading(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
        VecBuilder.fill(65, 65, Units.degreesToRadians(75)));
  }

  @Override
  public void periodic() {
    // Update Gyro Log Inputs
    m_gyro.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", m_gyroInputs);

    // Update Swerve Module Log Inputs
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + i, m_moduleInputs[i]);
    }

    m_poseEstimator.update(getGyroHeading(), getModulePositions());
    Logger.getInstance().recordOutput("Odometry", getPose());
    Logger.getInstance().recordOutput("ModuleStates", getModuleStates());
    FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
        DriveConstants.kSwerveModuleTranslations);
  }

  public void drive(ChassisSpeeds speeds) {
    Logger.getInstance().recordOutput(
        "DesiredSpeeds",
        new double[] {
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
        });

    if (speeds.vxMetersPerSecond == 0.0 && speeds.vyMetersPerSecond == 0.0 && speeds.omegaRadiansPerSecond == 0.0) {
      brake();
      return;
    }

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState... desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //     desiredStates,
    //     DriveConstants.kMaxSpeedMetersPerSecond);

    Logger.getInstance().recordOutput("DesiredModuleStates", desiredStates);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];

    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];

    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }

    return positions;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Rotation2d getGyroHeading() {
    return m_gyro.getRotation2d();
  }

  public void brake() {
    for (SwerveModuleIO module : m_modules) {
      module.setDesiredState(new SwerveModuleState(0, module.getRotation()));
    }
  }

  public void setTurnBrakeMode(boolean brake) {
    for (SwerveModuleIO module : m_modules) {
      module.setTurnBrakeMode(brake);
    }
  }

  public void setDriveBrakeMode(boolean brake) {
    for (SwerveModuleIO module : m_modules) {
      module.setDriveBrakeMode(brake);
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroHeading(), getModulePositions(), pose);
  }

  public SwerveModuleIO[] getModules() {
    return m_modules;
  }

  public GyroIO getGyro() {
    return m_gyro;
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  //#region Sim Stuff

  private double m_simGyroLastUpdated;

  @Override
  public void simulationPeriodic() {
    double gyroDelta = getChassisSpeeds().omegaRadiansPerSecond;
    double ts = Timer.getFPGATimestamp();

    double deltaTime = ts - m_simGyroLastUpdated;

    getGyro().addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
    m_simGyroLastUpdated = ts;
  }

  //#endregion

  //#region Commands

  public CommandBase forwardCommand(double speed) {
    var speeds = new ChassisSpeeds(speed, 0, 0);
    return runEnd(() -> drive(speeds), this::brake);
  }

  public CommandBase brakeCommand() {
    return runOnce(this::brake);
  }

  public CommandBase hardBrakeCommand() {
    return runOnce(() -> {
      this.setTurnBrakeMode(true);
      this.setDriveBrakeMode(true);
      this.brake();
    });
  }

  public CommandBase setBrakeModeCommand(boolean driveEnabled, boolean turnEnabled) {
    return runOnce(() -> {
      setDriveBrakeMode(driveEnabled);
      setTurnBrakeMode(turnEnabled);
    });
  }

  public CommandBase setBrakeModeCommand(boolean enabled) {
    return setBrakeModeCommand(enabled, enabled);
  }

  public CommandBase resetPoseCommand(Pose2d pose) {
    return runOnce(() -> resetPose(pose));
  }

  public CommandBase resetPoseCommand(Supplier<Pose2d> pose) {
    return runOnce(() -> resetPose(pose.get()));
  }

  //#endregion
}

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIO;
import frc.robot.subsystems.drive.accelerometer.AccelerometerInputsAutoLogged;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.util.SecondOrderKinematics;
import frc.robot.util.SwerveModuleAcceleration;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleInputsAutoLogged[] m_inputs;

  //SOK vars
  private final SecondOrderKinematics m_SecondOrderKinematics;
  private SwerveModuleAcceleration[] m_moduleAccelerations = new SwerveModuleAcceleration[] {
      new SwerveModuleAcceleration(), new SwerveModuleAcceleration(), new SwerveModuleAcceleration(),
      new SwerveModuleAcceleration() };
  private Rotation2d[] m_moduleSteerThetaVels = new Rotation2d[] { new Rotation2d(), new Rotation2d(), new Rotation2d(),
      new Rotation2d() };
  private Rotation2d[] m_moduleSteerOldTheta = new Rotation2d[] { new Rotation2d(), new Rotation2d(), new Rotation2d(),
      new Rotation2d() };
  private Rotation2d m_oldRobotTheta = new Rotation2d();
  private Rotation2d m_robotThetaVel = new Rotation2d();

  private final AccelerometerIO m_accel;
  private final AccelerometerInputsAutoLogged m_accelInputs;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final GyroIO m_gyro;
  private final GyroInputsAutoLogged m_gyroInputs;

  private final double[] m_lockAngles = new double[] { 45, 315, 45, 315 };

  private final double m_deltaTime = 0.02;

  private double m_simGyroLastUpdated;

  /** Creates a new Drive. */
  public Drive(GyroIO gyro, AccelerometerIO accel, Pose2d startPose, SwerveModuleIO... modules) {
    m_modules = modules;
    m_gyro = gyro;
    m_gyroInputs = new GyroInputsAutoLogged();
    for (SwerveModuleIO module : m_modules) {
      module.resetDistance();
      module.syncTurningEncoder();
      // module.resetEncoders();
    }

    m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleInputsAutoLogged();
    }
    m_poseEstimator = new SwerveDrivePoseEstimator(
        Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(), getSwerveModulePositions(), startPose);

    m_accel = accel;
    m_accelInputs = new AccelerometerInputsAutoLogged();
    m_SecondOrderKinematics = new SecondOrderKinematics();
  }

  @Override
  public void periodic() {

    updateSOKVars(m_deltaTime);

    //Update Gyro Inputs/Logs
    m_gyro.updateInputs(m_gyroInputs);
    Logger.getInstance().processInputs("Gyro", m_gyroInputs);

    //Update Swerve Module Inputs/Logs
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
    }

    // Update SOK Log Inputs
    for (int i = 0; i < m_modules.length; i++) {
      Logger.getInstance().recordOutput("Drive/SOK/ModuleAccels" + i, m_moduleAccelerations[i].getAccel());
    }

    m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());

    addAccelAndSOK();

    Logger.getInstance().recordOutput("Drive/Pose", getPose());
    Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
    FieldUtil.getDefaultField().setSwerveRobotPose(getPose(), getModuleStates(),
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
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private void updateSOKVars(double deltaTime) {
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < m_modules.length; i++) {
      //update moduleAccels and m_moduleSteerThetaVels
      m_moduleAccelerations[i].calculate(moduleStates[i], deltaTime);
      m_moduleSteerThetaVels[i] = new Rotation2d(
          moduleStates[i].angle.minus(m_moduleSteerOldTheta[i]).getRadians() / deltaTime);
    }
    //update robotThetaVel
    m_robotThetaVel = new Rotation2d(m_gyro.getAngle().minus(m_oldRobotTheta).getRadians() / deltaTime);
  }

  public Pose2d getPose2dfromSOK(
      double deltaTime) {

    SwerveModuleState[] moduleStates = getModuleStates();
    double[] moduleVelocities = new double[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      moduleVelocities[i] = moduleStates[i].speedMetersPerSecond;
    }
    Rotation2d robotTheta = getPose().getRotation();
    ChassisSpeeds sokChassisSpeeds = DriveConstants.kDriveKinematics
        .toChassisSpeeds(m_SecondOrderKinematics.getModuleStatesFromAccelXY(
            m_moduleAccelerations,
            moduleStates,
            m_moduleSteerThetaVels,
            moduleVelocities,
            m_robotThetaVel,
            robotTheta,
            deltaTime));
    return getPoseEstimator().getEstimatedPosition().exp(new Twist2d(
        sokChassisSpeeds.vxMetersPerSecond * deltaTime,
        sokChassisSpeeds.vyMetersPerSecond * deltaTime,
        sokChassisSpeeds.omegaRadiansPerSecond * deltaTime));
  }

  private void addAccelAndSOK() {

    //Implement Accelerometer and SOK into pose calculations

    Twist2d twist = new Twist2d(m_accelInputs.accelX, m_accelInputs.accelY, 0);

    Pose2d accelEstPose = getPoseEstimator().getEstimatedPosition().exp(twist);

    Logger.getInstance().recordOutput("Drive/Accel/EstimatedPose", accelEstPose);

    m_poseEstimator.addVisionMeasurement(
        accelEstPose,
        Timer.getFPGATimestamp(),
        VecBuilder.fill(30, 30, Units.degreesToRadians(1000)));

    Pose2d sokEstPose = getPose2dfromSOK(0.02); //deltaTime finally chosen to be tick time xd

    Logger.getInstance().recordOutput("Drive/SOK/Estimatedpose", sokEstPose);

    //commented out for future testing of other fixes to figure-8s

    m_poseEstimator.addVisionMeasurement(
        sokEstPose,
        Timer.getFPGATimestamp(),
        VecBuilder.fill(DriveConstants.kSOKStDevX.get(), DriveConstants.kSOKStDevY.get(),
            Units.degreesToRadians(DriveConstants.kSOKStDevTheta.get())));
  }

  public void resetOdometry() {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(),
        new Pose2d());//1.80, 1.14, new Rotation2d()
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
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

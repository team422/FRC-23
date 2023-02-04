package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.FieldUtil;

public class FullSwerveBase extends SubsystemBase {
  /**
   * Creates a new this.
   */
  // these should be in the order front left, front right, back left, back right
  SwerveModule m_swerveModules[] = new SwerveModule[4];
  Gyro m_gyro;
  SwerveDrivePoseEstimator m_odometry;
  // Swerve Drive Odometry with vision correction
  SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  String m_swerveModuleNames[] = { "Left Front", "Right Front", "Left Rear", "Right Rear" };
  Double m_swerveModuleStopAngles[] = { 45.0, 135.0, -45.0, -135.0 };

  //target pose and controller
  Pose2d m_targetPose;
  PIDController m_thetaController = new PIDController(1.0, 0.0, 0.05);
  int m_currentWheel = 0;
  Boolean m_singleWheelMode = false;
  double max_speed = 0;

  public FullSwerveBase(SwerveModule[] swerveModules, Gyro gyro) {
    // Setting up all the modules

    this.m_swerveModules = swerveModules;
    for (SwerveModule module : m_swerveModules) {
      module.resetDistance();
      module.syncTurningEncoders();
      // module.DONTUSETHISRESETTURNINGENCODER();
      // module.resetEncoders();
    }
    for (SwerveModule module : m_swerveModules) {
      module.setDesiredState(new SwerveModuleState(0.0, module.getAbsoluteRotation()));
    }
    // m_targetPose = m_odometry.getPoseMeters();
    m_thetaController.reset();
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.m_gyro = gyro;
    m_gyro.calibrate();
    m_gyro.reset();

    // odometry stuff
    // SwerveModulePositions[] modulePositions = new SwerveModulePositions[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition(m_swerveModules[i].getDriveDistanceMeters(),
          m_swerveModules[i].getTurnDegrees());
    }
    // SwerveModulePositions[] modulePositions = new SwerveModulePositions[4];
    // Rotation2d gyroAngle,
    //   Pose2d initialPoseMeters,
    //   SwerveDriveKinematics kinematics,
    //   Matrix<N3, N1> stateStdDevs,
    //   Matrix<N1, N1> localMeasurementStdDevs,
    //   Matrix<N3, N1> visionMeasurementStdDevs)
    // m_odometry = new SwerveDrivePoseEstimator(this.getHeading(), new Pose2d(), DriveConstants.kDriveKinematics,stdDeviations, stdDeviations, stdDeviations);
    m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        this.getHeading(),
        modulePositions,
        new Pose2d(),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
        VecBuilder.fill(50, 50, Units.degreesToRadians(100)));
    // // DriveConstants.kDriveKinematics,  
  }

  public double[] getVelocities() {
    return new double[] { m_swerveModules[0].getDriveVelocityMetersPerSecond(),
        m_swerveModules[1].getDriveVelocityMetersPerSecond(), m_swerveModules[2].getDriveVelocityMetersPerSecond(),
        m_swerveModules[3].getDriveVelocityMetersPerSecond() };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /* 
    */

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = new SwerveModulePosition(m_swerveModules[i].getDriveDistanceMeters(),
          m_swerveModules[i].getTurnDegrees());
    }
    m_odometry.update(getGyroAngle(), modulePositions);
    // m_odometry.update(this.getHeading(), m_swerveModules[0].getState(), m_swerveModules[1].getState(),
    //         m_swerveModules[2].getState(), m_swerveModules[3].getState());
    //This was literally all copy paste, these should be good for debugging
    // SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    // SmartDashboard.putNumber("currentX", getPose().getX());
    // SmartDashboard.putNumber("currentY", getPose().getY());
    // SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());

    // SmartDashboard.putNumber("Left Front Absolute", m_swerveModules[0].getAbsoluteRotation().getDegrees());
    // SmartDashboard.putNumber("Right Front Absolute", m_swerveModules[1].getAbsoluteRotation().getDegrees());
    // SmartDashboard.putNumber("Left Rear Absolute", m_swerveModules[2].getAbsoluteRotation().getDegrees());
    // SmartDashboard.putNumber("Right Rear Absolute", m_swerveModules[3].getAbsoluteRotation().getDegrees());
    // SmartDashboard.putNumber("Left Front encoder", m_swerveModules[0].getTurnDegrees().getDegrees());
    // SmartDashboard.putNumber("Right Front encoder", m_swerveModules[1].getTurnDegrees().getDegrees());
    // SmartDashboard.putNumber("Left Rear encoder", m_swerveModules[2].getTurnDegrees().getDegrees());
    // SmartDashboard.putNumber("Right Rear encoder", m_swerveModules[3].getTurnDegrees().getDegrees());
    // if (Math.abs(m_swerveModules[0].getDriveVelocityMetersPerSecond()) > max_speed) {
    //   max_speed = Math.abs(m_swerveModules[0].getDriveVelocityMetersPerSecond());
    //   SmartDashboard.putNumber("VELOCITY STRAIGHT", max_speed);
    // }
    SmartDashboard.putNumber("cur velocity", m_swerveModules[0].getDriveVelocityMetersPerSecond());

    SmartDashboard.putNumber(" ODO X", getPose().getX());
    SmartDashboard.putNumber(" ODO Y", getPose().getY());
    // SmartDashboard.putNumber(" ODO Angle", getPose().getRotation().getDegrees());
    // SmartDashboard.putNumber("Convofact", getPose().getX() / ModuleConstants.kDriveConversionFactor);
    FieldUtil.getDefaultField().setSwerveRobotPose(this.getPose(), this);

    // SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());
    /*
    for (int i = 0; i < m_swerveModules.length; i++) {
        SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Drive Speed",
                m_swerveModules[i].getDriveVelocityMetersPerSecond());
        SmartDashboard.putNumber(m_swerveModuleNames[i] + ": Rotation",
                m_swerveModules[i].getTurnDegrees());
    }
     */
  }

  // returns estimated position based on odometry
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void printAllVals() {
    // for (SwerveModule iModule : this.m_swerveModules) {
    //     System.out.println(""+iModule.getState());
    // }
    for (int i = 0; i < 4; i++) {
      // System.out.println(m_swerveModuleNames[i] + m_swerveModules[i].getState());
    }
  }

  public void switchWheel() {
    if (m_currentWheel == 3) {
      m_currentWheel -= 4;
    }
    m_currentWheel = m_currentWheel + 1;

  }

  public void switchTestingMode() {
    m_singleWheelMode = !m_singleWheelMode;
  }

  // returns the heading of the robot
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public void brake() {
    for (SwerveModule module : m_swerveModules) {
      module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
    }
  }

  public void fullBrake() {
    int moduleNum = 0;
    for (SwerveModule module : m_swerveModules) {
      module.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(m_swerveModuleStopAngles[moduleNum])));
      moduleNum += 1;
    }
  }

  public void drive(ChassisSpeeds speeds) {
    // SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveModuleState[] moduleStatesFinal = new SwerveModuleState[4];
    if (speeds.omegaRadiansPerSecond == 0 && speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0) {
      this.brake();
    } else {
      for (int i = 0; i < 4; i++) {
        moduleStatesFinal[i] = SwerveModuleState.optimize(moduleStates[i],
            m_swerveModules[i].getTurnDegrees());
      }
      if (!this.m_singleWheelMode) {
        // SwerveModule.normalizeWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        setModuleStates(moduleStatesFinal);
      } else {
        m_swerveModules[this.m_currentWheel]
            .setDesiredState(moduleStates[this.m_currentWheel]);
        // System.out.println("Wanted:" + moduleStates[this.m_currentWheel]);
        // System.out.println("Actual Degree:" + m_swerveModules[this.m_currentWheel].getTurnDegrees());
        // System.out.println("Actual" + m_swerveModules[this.m_currentWheel].getState());
      }
    }

  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    for (int i = 0; i < 4; i++) {
      m_swerveModules[i].setDesiredState(moduleStates[i]);
    }
  }

  public void setDesiredTurn(SwerveModuleState state) {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = state;
    }
    setModuleStates(moduleStates);
  }

  public Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
  }

  public void addVisionOdometry(Pose3d pose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(pose.toPose2d(),
        timestampSeconds);
  }

  public SwerveModuleState[] getSwerveStates() {
    return new SwerveModuleState[] { m_swerveModules[0].getState(), m_swerveModules[1].getState(),
        m_swerveModules[2].getState(), m_swerveModules[3].getState() };
  }

  public SwerveModulePosition[] getSwervePositions() {
    return new SwerveModulePosition[] { m_swerveModules[0].getPosition(), m_swerveModules[1].getPosition(),
        m_swerveModules[2].getPosition(), m_swerveModules[3].getPosition() };
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(getGyroAngle(), getSwervePositions(), pose);

  }

  public CommandBase fullBrakeCommand() {
    return Commands.none();
    // return runOnce(this::brake);

  }
}

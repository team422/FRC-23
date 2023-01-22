package frc.robot.subsystems.drive.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleIOMK2Neo implements SwerveModuleIO {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turnMotor;

  private final RelativeEncoder m_driveMotorEncoder;
  private final RelativeEncoder m_turnMotorEncoder;

  private final Rotation2d m_absoluteOffset;

  private final AnalogEncoder m_turnAbsoluteEncoder;

  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turnPIDController;

  public SwerveModuleIOMK2Neo(int turnMotorPort, int driveMotorPort, int turnAbsoluteEncoderPort, Rotation2d offset) {
    // Define Motors
    this.m_turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
    this.m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);

    this.m_absoluteOffset = offset;

    // Set Idle Modes
    this.m_turnMotor.setIdleMode(IdleMode.kBrake);
    this.m_driveMotor.setIdleMode(IdleMode.kCoast);

    m_turnMotor.setSmartCurrentLimit(30);
    m_driveMotor.setSmartCurrentLimit(30);

    m_turnMotor.setInverted(false);
    m_driveMotor.setInverted(false);

    // Define Encoders
    m_turnMotorEncoder = m_turnMotor.getEncoder();
    m_driveMotorEncoder = m_driveMotor.getEncoder();

    m_turnAbsoluteEncoder = new AnalogEncoder(turnAbsoluteEncoderPort);
    m_turnAbsoluteEncoder.setDistancePerRotation(DriveConstants.kTurnPositionConversionFactor);

    // Define Encoder Conversion Factors
    m_turnMotorEncoder.setPositionConversionFactor(DriveConstants.kTurnPositionConversionFactor); // radians
    m_turnMotorEncoder.setVelocityConversionFactor(DriveConstants.kTurnPositionConversionFactor / 60); // radians per second

    m_driveMotorEncoder.setPositionConversionFactor(DriveConstants.kDrivePositionConversionFactor); // meters
    m_driveMotorEncoder.setVelocityConversionFactor(DriveConstants.kDrivePositionConversionFactor / 60); // meters per second

    // Define PID Controllers
    m_turnPIDController = m_turnMotor.getPIDController();
    m_drivePIDController = m_driveMotor.getPIDController();

    m_turnPIDController.setPositionPIDWrappingEnabled(true);
    m_turnPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_turnPIDController.setPositionPIDWrappingMaxInput(Math.PI);

    setTurnPID(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
    setDrivePID(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput();
    inputs.drivePositionMeters = getDrivePositionMeters();
    inputs.driveVelocityMetersPerSec = getVelocityMetersPerSecond();
    inputs.driveCurrentAmps = m_driveMotor.getOutputCurrent();
    inputs.driveTempCelcius = m_driveMotor.getMotorTemperature();

    inputs.turnAppliedVolts = m_turnMotor.getAppliedOutput();
    inputs.turnAbsolutePositionRad = getAbsoluteRotation().getRadians();
    inputs.turnPositionRad = getRotation().getRadians();
    inputs.turnVelocityRadPerSec = m_turnMotorEncoder.getVelocity();
    inputs.turnCurrentAmps = m_turnMotor.getOutputCurrent();
    inputs.turnTempCelcius = m_turnMotor.getMotorTemperature();
  }

  @Override
  public double getVelocityMetersPerSecond() {
    return m_driveMotorEncoder.getVelocity();
  }

  @Override
  public Rotation2d getRotation() {
    double angle = MathUtil.angleModulus(m_turnMotorEncoder.getPosition());
    return new Rotation2d(angle);
  }

  @Override
  public Rotation2d getAbsoluteRotation() {
    double angle = (1.0 - m_turnAbsoluteEncoder.getAbsolutePosition()) * 2 * Math.PI;
    return new Rotation2d(MathUtil.angleModulus(angle)).plus(m_absoluteOffset);
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    var optimized = SwerveModuleState.optimize(state, getRotation());
    m_turnPIDController.setReference(optimized.angle.getRadians(), ControlType.kPosition);

    // Set Drive Motor Setpoint
    m_drivePIDController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity, 0,
        DriveConstants.kDriveFF * optimized.speedMetersPerSecond);

  }

  @Override
  public double getDrivePositionMeters() {
    return m_driveMotorEncoder.getPosition();
  }

  @Override
  public void zeroTurnEncoder() {
    m_turnMotorEncoder.setPosition(0.0);
  }

  @Override
  public void zeroDriveEncoder() {
    m_driveMotorEncoder.setPosition(0.0);
  }

  @Override
  public void syncTurnEncoderWithAbsolute() {
    m_turnMotorEncoder.setPosition(getAbsoluteRotation().getRadians());
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    m_turnMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    m_driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnPID(double p, double i, double d) {
    m_turnPIDController.setP(p);
    m_turnPIDController.setI(i);
    m_turnPIDController.setD(d);
  }

  @Override
  public void setDrivePID(double p, double i, double d) {
    m_drivePIDController.setP(p);
    m_drivePIDController.setI(i);
    m_drivePIDController.setD(d);
  }
}

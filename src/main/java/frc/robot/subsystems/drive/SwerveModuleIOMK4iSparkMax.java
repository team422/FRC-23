package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.CanSparkMaxSetup;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class SwerveModuleIOMK4iSparkMax implements SwerveModuleIO {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final CANCoder m_turningCANCoder; // THIS MAY HAVE TO BE CHANGED BACK TO AN ANALOG ENCODER
  // absolute offset for the CANCoder so that the wheels can be aligned when the
  // robot is turned on
  //    private final Rotation2d m_CANCoderOffset;

  private final SparkMaxPIDController m_turningController;
  private final SparkMaxPIDController m_driveController;

  private double adjustedSpeed;
  private String name;

  public static class ModuleConstants {
    public static final double kDriveConversionFactor = 1 / 22.0409;
    public static final double kTurnPositionConversionFactor = 21.428;
    public static final TunableNumber kDriveP = Constants.ModuleConstants.kDriveP;
    public static final TunableNumber kDriveI = Constants.ModuleConstants.kDriveI;
    public static final TunableNumber kDriveD = Constants.ModuleConstants.kDriveD;
    public static final TunableNumber kTurningP = Constants.ModuleConstants.kTurningP;
    public static final TunableNumber kTurningI = Constants.ModuleConstants.kTurningI;
    public static final TunableNumber kTurningD = Constants.ModuleConstants.kTurningD;
    // public static final TunableNumber kDriveFF = RobotContainer.robotConstants.kDriveFF;

  }

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleIOMK4iSparkMax(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningCANCoderChannel) {
    CanSparkMaxSetup setup = new CanSparkMaxSetup();
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    // m_driveMotor.burnFlash();
    m_driveMotor.restoreFactoryDefaults();
    setup.setupSparkMaxSlow(m_driveMotor);
    m_driveMotor.setInverted(true);
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    // m_turningMotor.burnFlash();
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    // m_driveMotor.burnFlash();
    // m_turningMotor.burnFlash();
    setup.setupSparkMaxSlow(m_turningMotor);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_turningCANCoder = new CANCoder(turningCANCoderChannel);

    m_turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningCANCoder.configSensorDirection(false);

    // m_driveEncoder returns RPM by default. Use setVelocityConversionFactor() to
    // convert that to meters per second.
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

    m_turningEncoder.setVelocityConversionFactor((360.0 / ModuleConstants.kTurnPositionConversionFactor) / 60.0);
    m_turningEncoder.setPositionConversionFactor((360.0 / ModuleConstants.kTurnPositionConversionFactor)); // FIX THIS LATER ****
    m_turningMotor.setInverted(true);

    m_turningController = m_turningMotor.getPIDController();
    m_driveController = m_driveMotor.getPIDController();

    m_driveMotor.enableVoltageCompensation(12);
    m_driveMotor.setSmartCurrentLimit(60);
    m_turningMotor.setSmartCurrentLimit(60);
    m_turningMotor.enableVoltageCompensation(12);

    m_turningController.setPositionPIDWrappingEnabled(true);
    m_turningController.setPositionPIDWrappingMinInput(-180);
    m_turningController.setPositionPIDWrappingMaxInput(180);

    m_turningController.setP(ModuleConstants.kTurningP.get());
    m_turningController.setI(ModuleConstants.kTurningI.get());
    m_turningController.setD(ModuleConstants.kTurningD.get());
    // setDesiredState(new SwerveModuleState(0, new Rotation2d(m_turningCANCoder.getAbsolutePosition() * 2)));
    // // 401 only sets P of the drive PID
    m_driveController.setP(ModuleConstants.kDriveP.get());
    m_driveController.setI(ModuleConstants.kDriveI.get());
    m_driveController.setD(ModuleConstants.kDriveD.get());
  }

  public String getName() {
    return name;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }

  @Override
  public SwerveModuleState getAbsoluteState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAbsoluteRotation());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), getAngle());
  }

  public CANSparkMax getTurnMotor() {
    return m_turningMotor;
  }

  public RelativeEncoder getTurnEncoder() {
    return m_turningEncoder;
  }

  public double getAbsoluteEncoder() {
    return m_turningCANCoder.getAbsolutePosition();
  }

  public Rotation2d getAbsoluteRotation() {
    // double angle = (Math.abs((1.0 - m_turningCANCoder.getAbsolutePosition() / 180)) * 2 * Math.PI); FOR ANALOG
    return Rotation2d.fromDegrees(m_turningCANCoder.getAbsolutePosition());
  }

  public Rotation2d adjustedAngle = new Rotation2d();

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed (in meters per second?) and angle (in
   *              degrees).
   */
  @Override
  public void setDesiredState(SwerveModuleState state) {
    double driveOutput = state.speedMetersPerSecond;
    m_turningController.setReference(
        state.angle.getDegrees(),
        ControlType.kPosition);

    adjustedSpeed = driveOutput;
    m_driveController.setReference(driveOutput, ControlType.kVelocity, 0,
        adjustedSpeed);
    if (Constants.tuningMode) {
      // m_turningController.setP(ModuleConstants.kTurningP.get());
      // m_turningController.setI(ModuleConstants.kTurningI.get());
      // m_turningController.setD(ModuleConstants.kTurningD.get());
      // // setDesiredState(new SwerveModuleState(0, new Rotation2d(m_turningCANCoder.getAbsolutePosition() * 2)));
      // // // 401 only sets P of the drive PID
      // m_driveController.setP(ModuleConstants.kDriveP.get());
      // m_driveController.setI(ModuleConstants.kDriveI.get());
      // m_driveController.setD(ModuleConstants.kDriveD.get());
    }
  }

  //calculate the angle motor setpoint based on the desired angle and the current angle measurement
  // Arguments are in radians.
  public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
    return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
  }

  public double getDriveDistanceMeters() {
    return m_driveEncoder.getPosition();
  }

  public void resetDistance() {
    m_driveEncoder.setPosition(0.0);
  }

  @Override
  public void syncTurningEncoder() {
    m_turningEncoder.setPosition(getAbsoluteRotation().getDegrees());
  }

  @Override
  public void setUpModuleFirmware() {
    m_driveMotor.setInverted(true);
    m_turningMotor.setInverted(true);
    // m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
    // m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

    // m_turningEncoder.setVelocityConversionFactor((360.0 / ModuleConstants.kTurnPositionConversionFactor) / 60.0);
    // m_turningEncoder.setPositionConversionFactor((360.0 / ModuleConstants.kTurnPositionConversionFactor)); // FIX THIS LATER ****
  }

  /** Zeros all the SwerveModule encoders. */
  @Override
  public void resetEncoders() {
    // Reset the cumulative rotation counts of the SparkMax motors
    m_turningEncoder.setPosition(0.0);

    m_turningCANCoder.setPosition(0.0);
    m_turningCANCoder.configMagnetOffset(
        m_turningCANCoder.configGetMagnetOffset() - m_turningCANCoder.getAbsolutePosition());
  }

  public double getDriveVelocityMetersPerSecond() {
    return m_driveEncoder.getVelocity();
  }

  @Override
  public Rotation2d getAngle() {
    double angle = Units.degreesToRadians(m_turningEncoder.getPosition());
    return new Rotation2d(MathUtil.angleModulus(angle));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // TODO Auto-generated method stub
    inputs.driveDistanceMeters = getDriveDistanceMeters();
    inputs.driveVelocityMetersPerSecond = getDriveVelocityMetersPerSecond();
    inputs.turnAngleRads = getAngle().getRadians();
    inputs.turnRadsPerSecond = Units.degreesToRadians(m_driveEncoder.getVelocity());
  }

}

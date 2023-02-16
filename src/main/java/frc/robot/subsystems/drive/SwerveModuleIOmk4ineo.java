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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.TunableNumber;

public class SwerveModuleIOmk4ineo implements SwerveModuleIO {

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

  private final double m_offset;
  private double m_turningDesiredPosition = 0;
  private double m_drivingDesiredSpeed = 0;
  private double adjustedSpeed;
  private String name;

  public static class ModuleConstants {
    public static final double kDriveConversionFactor = 1 / 22.0409;
    public static final double kTurnPositionConversionFactor = 21.428;
    public static final TunableNumber kDriveP = RobotContainer.robotConstants.kDriveP;
    public static final TunableNumber kDriveI = RobotContainer.robotConstants.kDriveI;
    public static final TunableNumber kDriveD = RobotContainer.robotConstants.kDriveD;
    public static final TunableNumber kTurningP = RobotContainer.robotConstants.kTurningP;
    public static final TunableNumber kTurningI = RobotContainer.robotConstants.kTurningI;
    public static final TunableNumber kTurningD = RobotContainer.robotConstants.kTurningD;
    // public static final TunableNumber kDriveFF = RobotContainer.robotConstants.kDriveFF;

  }

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleIOmk4ineo(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningCANCoderChannel,
      double offset) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setInverted(true);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();

    m_turningCANCoder = new CANCoder(turningCANCoderChannel);

    m_turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningCANCoder.configSensorDirection(false);
    // m_turningCANCoder.setPosition(0);

    m_offset = offset;

    // m_CANCoderOffset = Rotation2d.fromDegrees(turningCANCoderOffsetDegrees);
    // Add smart dashboard items
    // SmartDashboard.putNumber("Drive Offset " + turningCANCoderChannel, getAbsoluteRotation().getDegrees());

    // m_driveMotor.setIdleMode(IdleMode.kBrake);
    // m_turningMotor.setIdleMode(IdleMode.kCoast);

    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

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
    m_driveMotor.setSmartCurrentLimit(40);
    m_turningMotor.setSmartCurrentLimit(40);
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

  public void periodic() {
    SmartDashboard.putNumber("Drive Velocity " + getName(), m_driveEncoder.getVelocity());
    // SmartDashboard.putNumber("Drive Position " + getName(), m_driveEncoder.getPosition());
    SmartDashboard.putNumber("Drive Expected Speed " + getName(), adjustedSpeed);

    SmartDashboard.putNumber("Turn Expected Position " + getName(), m_turningDesiredPosition);
    SmartDashboard.putNumber("Turn Position " + getName(), m_turningEncoder.getPosition());
    if (ModuleConstants.kTurningP.hasChanged() || ModuleConstants.kTurningI.hasChanged()
        || ModuleConstants.kTurningD.hasChanged()) {
      m_turningController.setP(ModuleConstants.kTurningP.get());
      m_turningController.setI(ModuleConstants.kTurningI.get());
      m_turningController.setD(ModuleConstants.kTurningD.get());
    }
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
    // getPosition() returns the number of cumulative rotations.
    // Convert that to 0.0 to 1.0
    // double m1 = m_turningEncoder.getPosition() % 360.0;
    // double m2 = (m1 < 0) ? m1 + 360 : m1;

    // double m2 = (this.getTurnDegrees() % 360 + 360) % 360;

    return new SwerveModuleState(m_driveEncoder.getVelocity(), this.getTurnDegrees());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), this.getTurnDegrees());
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

  // public CANCoder getTurnCANcoder() {
  //     return m_turningCANCoder;
  // }

  // public double getTurnCANcoderAngle() {
  //     return m_turningCANCoder.getAbsolutePosition();
  // }

  public Rotation2d adjustedAngle = new Rotation2d();

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed (in meters per second?) and angle (in
   *              degrees).
   */
  public void setDesiredState(SwerveModuleState state) {

    Rotation2d curAngle = this.getTurnDegrees();

    // double delta = deltaAdjustedAngle((state.angle.getDegrees()) % 360, curAngle.getDegrees());

    // // Calculate the drive motor output from the drive PID controller.
    double driveOutput = state.speedMetersPerSecond;

    // if (Math.abs(delta) > 90) {
    //     driveOutput *= -1;
    //     delta -= Math.signum(delta) * 180;
    // }

    // adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());
    // if (Math.abs(curAngle.getDegrees() - state.angle.getDegrees()) < 45) {
    //     state.angle = curAngle;
    // }
    m_turningController.setReference(
        state.angle.getDegrees(),
        ControlType.kPosition);
    // m_turningController.setReference(state.angle.getDegrees(), ControlType.kPosition);
    m_turningDesiredPosition = state.angle.getDegrees();
    // SmartDashboard.putNumber("Desired Angle " + this.getName(), adjustedAngle.getDegrees());

    adjustedSpeed = driveOutput;
    // System.out.println(adjustedSpeed);
    // if (Math.abs(m_driveEncoder.getVelocity() - driveOutput) > 0.25) {
    //   adjustedSpeed = Math.max(Math.min(m_driveEncoder.getVelocity() - driveOutput, 0.25), -0.25)
    //       + m_driveEncoder.getVelocity();
    //   System.out
    //       .println("slip detected" + adjustedSpeed + " " + driveOutput + " " + m_driveEncoder.getVelocity());
    // }
    // System.out.println(adjustedSpeed);
    m_driveController.setReference(driveOutput, ControlType.kVelocity, 0,
        adjustedSpeed);
    if (Constants.tuningMode) {
      m_turningController.setP(ModuleConstants.kTurningP.get());
      m_turningController.setI(ModuleConstants.kTurningI.get());
      m_turningController.setD(ModuleConstants.kTurningD.get());
      // setDesiredState(new SwerveModuleState(0, new Rotation2d(m_turningCANCoder.getAbsolutePosition() * 2)));
      // // 401 only sets P of the drive PID
      m_driveController.setP(ModuleConstants.kDriveP.get());
      m_driveController.setI(ModuleConstants.kDriveI.get());
      m_driveController.setD(ModuleConstants.kDriveD.get());
    }
    m_drivingDesiredSpeed = adjustedSpeed;
  }

  public void setOpenLoopState(SwerveModuleState state) {
    Rotation2d curAngle = this.getTurnDegrees();

    // double delta = deltaAdjustedAngle(state.angle.getDegrees(), curAngle.getDegrees());

    // // Calculate the drive motor output from the drive PID controller.
    // double driveOutput = state.speedMetersPerSecond;

    // if (Math.abs(delta) > 90) {
    //     driveOutput *= -1;
    //     delta -= Math.signum(delta) * 180;
    // }

    // adjustedAngle = Rotation2d.fromDegrees(delta + curAngle.getDegrees());

    // m_turningController.setReference(
    //         adjustedAngle.getDegrees(),
    //         ControlType.kPosition);

    // SmartDashboard.putNumber("Commanded Velocity", driveOutput);

    // m_driveMotor.setVoltage(Constants.ModuleConstants.kDriveFF * driveOutput);
  }

  //calculate the angle motor setpoint based on the desired angle and the current angle measurement
  // Arguments are in radians.
  public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
    // return targetAngle;
    return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
  }

  public double getDriveDistanceMeters() {
    return m_driveEncoder.getPosition();
  }

  public void resetDistance() {
    m_driveEncoder.setPosition(0.0);
  }

  public void syncTurningEncoders() {
    // System.out.println("SYNCING");
    // System.out.println("absolute position: " + m_turningCANCoder.getAbsolutePosition());
    // System.out.println("get correct degree " + getTurnDegrees());
    // System.out.println("m_turningCANCoder.getAbsolutePosition() - m_offset:"
    //         + (getAbsoluteRotation().getDegrees() - m_offset));
    // m_turningEncoder.setPosition(getAbsoluteRotation().getDegrees() - m_offset);

    // System.out.println("absolute position: " + m_turningCANCoder.getAbsolutePosition());
    // System.out.println("get correct degree " + getTurnDegrees());
    m_turningEncoder.setPosition(getAbsoluteRotation().getDegrees());
  }

  public void DONTUSETHISRESETTURNINGENCODER() {
    m_turningEncoder.setPosition(0);
  }

  /** Zeros all the SwerveModule encoders. */
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

  public Rotation2d getTurnDegrees() {
    double angle = Units.degreesToRadians(m_turningEncoder.getPosition());
    return new Rotation2d(MathUtil.angleModulus(angle));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // TODO Auto-generated method stub

  }

}

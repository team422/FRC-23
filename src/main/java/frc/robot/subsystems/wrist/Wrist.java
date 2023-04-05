package frc.robot.subsystems.wrist;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Wrist extends SubsystemBase {
  private double kMinAngle;
  private double kMaxAngle;
  private WristIO m_io;
  private WristInputsAutoLogged m_inputs;
  public Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);
  public Rotation2d userWantedAngle = Rotation2d.fromDegrees(0);
  private ProfiledPIDController m_controller;
  private ArmFeedforward m_feedforward;

  private double m_lastTime;
  private double m_lastVelocitySetpoint;

  public Wrist(WristIO io, ProfiledPIDController wristPIDController, ArmFeedforward feedForward, Rotation2d minAngle,
      Rotation2d maxAngle) {
    m_io = io;
    m_inputs = new WristInputsAutoLogged();

    m_controller = wristPIDController;
    m_feedforward = feedForward;
    m_controller.setTolerance(Units.degreesToRadians(2));
    kMinAngle = minAngle.getRadians();
    kMaxAngle = maxAngle.getRadians();
    m_controller.setIntegratorRange(-0.5, 0.5);

    // m_controller.reset(0);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Wrist", m_inputs);
    Double curAngle = m_inputs.angleRad;
    double dt = Timer.getFPGATimestamp() - m_lastTime;
    if (Robot.isSimulation()) {
      curAngle = -curAngle;
    }
    double pidVoltage = m_controller.calculate(curAngle, m_desiredAngle.getRadians());
    double positionSetpoint = m_controller.getSetpoint().position;
    // double positionSetpoint = m_controller.
    double velocitySetpoint = m_controller.getSetpoint().velocity;
    double accelerationSetpoint = (velocitySetpoint - m_lastVelocitySetpoint) / dt;
    double feedForwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint,
        accelerationSetpoint);

    double outputVoltage = pidVoltage + feedForwardVoltage;
    if (Robot.isSimulation()) {
      m_io.setVoltage(pidVoltage);
    } else {
      m_io.setVoltage(outputVoltage);

    }
    // m_io.setVoltage(feedForwardVoltage);

    // Logger.getInstance().recordOutput("Wrist/PIDVoltage", pidVoltage);
    // Logger.getInstance().recordOutput("Wrist/FFVoltage", feedForwardVoltage);
    // Logger.getInstance().recordOutput("Wrist/OutputVoltage", outputVoltage);
    Logger.getInstance().recordOutput("Wrist/SetpointDegrees", m_desiredAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/AngleDeg", Units.radiansToDegrees(m_inputs.angleRad));

    m_lastTime = Timer.getFPGATimestamp();
    m_lastVelocitySetpoint = velocitySetpoint;
  }

  public void reset() {
    setAngle(Rotation2d.fromRadians(m_inputs.angleRad));
    m_controller.reset(m_inputs.angleRad);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setAngle(Rotation2d angle) {
    m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
    userWantedAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
  }

  public void setAngleToNotBreak(Rotation2d angle) {
    m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_inputs.angleRad);
  }

  public Command setAngleCommand(Rotation2d angle) {
    return run(() -> setAngle(angle)).until(m_controller::atGoal);
  }

  public Command testSetAngleCommandOnce(Rotation2d angle) {
    return runOnce(() -> setAngle(angle));
  }

  public Command testSetAngleCommand(Rotation2d angle) {
    return testSetAngleCommand(angle, Units.degreesToRadians(3.0));
  }

  public Command testSetAngleCommand(Rotation2d angle, double toleranceRadians) {
    return Commands.sequence(
        testSetAngleCommandOnce(angle),
        Commands.waitSeconds(0.1),
        waitUntilWithinTolerance(toleranceRadians));
  }

  public Command waitUntilWithinTolerance(double toleranceRadians) {
    return Commands.waitUntil(() -> withinTolerance(toleranceRadians));
  }

  public boolean withinTolerance(double toleranceRadians) {
    return Math.abs(m_controller.getGoal().position - m_inputs.angleRad) < toleranceRadians;
  }

  public Command moveCommand(Supplier<Double> delta) {
    return run(() -> setAngle(m_desiredAngle.plus(Rotation2d.fromDegrees(delta.get()))));
  }

  public void setBrakeMode(boolean mode) {
    m_io.setBrakeMode(mode);
  }
}

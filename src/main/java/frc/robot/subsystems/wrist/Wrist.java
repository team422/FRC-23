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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private static final double kMinAngle = Units.degreesToRadians(-90);
  private static final double kMaxAngle = Units.degreesToRadians(90);
  private WristIO m_io;
  private WristInputsAutoLogged m_inputs;
  public Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);
  public Rotation2d userWantedAngle = Rotation2d.fromDegrees(0);
  private ProfiledPIDController m_controller;
  private ArmFeedforward m_feedforward;

  private double m_lastTime;
  private double m_lastVelocitySetpoint;

  public Wrist(WristIO io, ProfiledPIDController wristPIDController, ArmFeedforward feedForward) {
    m_io = io;
    m_inputs = new WristInputsAutoLogged();

    m_controller = wristPIDController;
    m_feedforward = feedForward;
    m_controller.setTolerance(Units.degreesToRadians(2));

    // m_controller.reset(0);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Wrist", m_inputs);

    double dt = Timer.getFPGATimestamp() - m_lastTime;

    double pidVoltage = m_controller.calculate(m_inputs.angleRad, m_desiredAngle.getRadians());
    double positionSetpoint = m_controller.getSetpoint().position;
    // double positionSetpoint = m_controller.
    double velocitySetpoint = m_controller.getSetpoint().velocity;
    double accelerationSetpoint = (velocitySetpoint - m_lastVelocitySetpoint) / dt;
    double feedForwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint,
        accelerationSetpoint);

    double outputVoltage = pidVoltage + feedForwardVoltage;

    m_io.setVoltage(outputVoltage);
    // m_io.setVoltage(feedForwardVoltage);

    Logger.getInstance().recordOutput("Wrist/PIDVoltage", pidVoltage);
    Logger.getInstance().recordOutput("Wrist/FFVoltage", feedForwardVoltage);
    Logger.getInstance().recordOutput("Wrist/OutputVoltage", outputVoltage);
    Logger.getInstance().recordOutput("Wrist/DesiredAngle", m_desiredAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/AngleDeg", Units.radiansToDegrees(m_inputs.angleRad));

    m_lastTime = Timer.getFPGATimestamp();
    m_lastVelocitySetpoint = velocitySetpoint;
  }

  public void reset() {
    setAngle(Rotation2d.fromRadians(m_inputs.angleRad));
    m_controller.reset(m_inputs.angleRad);
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
    return runOnce(() -> setAngle(angle));
  }

  public Command moveCommand(Supplier<Double> delta) {
    return run(() -> setAngle(m_desiredAngle.plus(Rotation2d.fromDegrees(delta.get()))));
  }
}

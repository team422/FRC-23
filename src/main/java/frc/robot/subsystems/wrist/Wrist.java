package frc.robot.subsystems.wrist;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    // if (WristConstants.kWristTuning) {
    //   if (WristConstants.kWristP.hasChanged() || WristConstants.kWristI.hasChanged()
    //       || WristConstants.kWristD.hasChanged() || WristConstants.kWristAccel.hasChanged()
    //       || WristConstants.kWristVelo.hasChanged()) {
    //     m_controller.setPID(WristConstants.kWristP.get(), WristConstants.kWristI.get(), WristConstants.kWristD.get());
    //     m_controller.setConstraints(new Constraints(WristConstants.kWristVelo.get(), WristConstants.kWristAccel.get()));
    //   }
    //   if (WristConstants.kWristSetpoint.hasChanged()) {
    //     setAngle(Rotation2d.fromDegrees(WristConstants.kWristSetpoint.get()));
    //   }
    //   if (WristConstants.kWristkg.hasChanged() || WristConstants.kWristks.hasChanged()
    //       || WristConstants.kWristkv.hasChanged() || WristConstants.kWristka.hasChanged()) {
    //     m_feedforward = new ArmFeedforward(WristConstants.kWristks.get(), WristConstants.kWristkg.get(),
    //         WristConstants.kWristkv.get(), WristConstants.kWristka.get());
    //   }
    // }
    m_io.updateInputs(m_inputs);
    // Logger.getInstance().processInputs("Wrist", m_inputs);

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

    // Logger.getInstance().recordOutput("Wrist/PIDVoltage", pidVoltage);
    // Logger.getInstance().recordOutput("Wrist/FFVoltage", feedForwardVoltage);
    // Logger.getInstance().recordOutput("Wrist/OutputVoltage", outputVoltage);
    // Logger.getInstance().recordOutput("Wrist/SetpointDegrees", m_desiredAngle.getDegrees());
    // Logger.getInstance().recordOutput("Wrist/AngleDeg", Units.radiansToDegrees(m_inputs.angleRad));

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
    return run(() -> setAngle(angle)).until(m_controller::atGoal);
  }

  public Command moveCommand(Supplier<Double> delta) {
    return run(() -> setAngle(m_desiredAngle.plus(Rotation2d.fromDegrees(delta.get()))));
  }

  public void setBrakeMode(boolean mode) {
    m_io.setBrakeMode(mode);
  }
}

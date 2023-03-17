package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final ElevatorInputsAutoLogged m_inputs;
  private final ElevatorIO m_io;
  // private final ElevatorInputs m_inputs;
  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_elevatorFeedForward;
  private double m_desiredHeight;
  private Rotation2d m_elevatorAngle;
  private double m_elevatorOffsetMeters;
  private double m_maxHeight;

  private double m_lastVelocity;
  private double m_lastTime;

  private boolean m_curZeroing;

  public Elevator(ElevatorIO io, ProfiledPIDController elevatorPIDController, ElevatorFeedforward elevatorFeedForward,
      double ElevatorOffsetMeters, double maxHeight, Rotation2d elevatorAngle) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();

    m_controller = elevatorPIDController;
    m_elevatorFeedForward = elevatorFeedForward;

    m_controller = Constants.ElevatorConstants.elevatorPIDController;
    m_controller.setTolerance(Units.inchesToMeters(0.3));
    m_elevatorFeedForward = Constants.ElevatorConstants.elevatorFeedForward;

    m_elevatorAngle = elevatorAngle;
    m_elevatorOffsetMeters = ElevatorOffsetMeters;
    m_maxHeight = maxHeight;
    m_lastVelocity = 0;
    m_lastTime = Timer.getFPGATimestamp();
    m_curZeroing = false;

  }

  public void periodic() {
    if (Constants.tuningMode) {
      if (ElevatorConstants.kP.hasChanged() || ElevatorConstants.kI.hasChanged()
          || ElevatorConstants.kD.hasChanged()) {
        m_controller.setP(ElevatorConstants.kP.get());
        m_controller.setI(ElevatorConstants.kI.get());
        m_controller.setD(ElevatorConstants.kD.get());
      }
      if (ElevatorConstants.kKs.hasChanged() || ElevatorConstants.kKv.hasChanged()
          || ElevatorConstants.kKg.hasChanged()) {
        m_elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kKs.get(),
            ElevatorConstants.kKg.get(), ElevatorConstants.kKv.get());
      }
    }
    if (ElevatorConstants.kTuningMode && ElevatorConstants.kManualSetpoint.hasChanged()) {
      setHeight(Units.inchesToMeters(ElevatorConstants.kManualSetpoint.get()));
    }

    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Elevator", m_inputs);

    double dt = Timer.getFPGATimestamp() - m_lastTime;

    double pidVoltage = m_controller.calculate(m_inputs.heightMeters, m_desiredHeight);
    double velocitySetpoint = m_controller.getSetpoint().velocity;
    double accelerationSetpoint = (velocitySetpoint - m_lastVelocity) / dt;
    double feedForwardVoltage = m_elevatorFeedForward.calculate(velocitySetpoint,
        accelerationSetpoint);

    double outputVoltage = pidVoltage + feedForwardVoltage;
    if (m_curZeroing) {
      m_io.setVoltage(-3);
      m_io.zeroHeight();
    } else {
      m_io.setVoltage(outputVoltage);
    }
    // m_io.setVoltage(feedForwardVoltage);

    Logger.getInstance().recordOutput("Elevator/PIDVoltage", pidVoltage);
    Logger.getInstance().recordOutput("Elevator/FFVoltage", feedForwardVoltage);
    Logger.getInstance().recordOutput("Elevator/OutputVoltage", outputVoltage);

    Logger.getInstance().recordOutput("Elevator/SetpointInches",
        Units.metersToInches(convertTravelToReal(m_desiredHeight)));
    Logger.getInstance().recordOutput("Elevator/HeightInches", Units.metersToInches(getPositionYMeters()));
    Logger.getInstance().recordOutput("Elevator/XInches", Units.metersToInches(getPositionXMeters()));

    m_lastVelocity = velocitySetpoint;
    m_lastTime = Timer.getFPGATimestamp();

  }

  public double getCurrentHeightMeters() {
    return m_inputs.heightMeters;
  }

  public double getDesiredMeters() {
    return convertTravelToReal(m_desiredHeight);
  }

  public double getPositionYMeters() {
    return m_inputs.heightMeters * Math.cos(m_elevatorAngle.getRadians()) + m_elevatorOffsetMeters;
  }

  public double getPositionXMeters() {
    return m_inputs.heightMeters * Math.sin(m_elevatorAngle.getRadians());
  }

  public double getPositionXMetersAtHeight(double heightMeters) {
    return heightMeters * Math.sin(m_elevatorAngle.getRadians());
  }

  public double getPositionYMetersAtElevatorX(double xMeters) {
    return (xMeters - m_elevatorOffsetMeters) / Math.tan(m_elevatorAngle.getRadians());
  }

  public double getTravelDistanceMeters() {
    return m_inputs.heightMeters;
  }

  public void setHeight(double heightMeters) {
    heightMeters = MathUtil.clamp(heightMeters, m_elevatorOffsetMeters, m_maxHeight);
    heightMeters -= m_elevatorOffsetMeters;
    heightMeters /= Math.cos(m_elevatorAngle.getRadians());
    m_desiredHeight = heightMeters;
  }

  public void reset() {
    setHeight(convertTravelToReal(m_inputs.heightMeters));
    m_controller.reset(m_inputs.heightMeters);
  }

  public double convertTravelToReal(double heightMeters) {
    heightMeters *= Math.cos(m_elevatorAngle.getRadians());
    heightMeters += m_elevatorOffsetMeters;
    return heightMeters;
  }

  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setHeight(heightMeters));
  }

  public Command moveCommand(Supplier<Double> heightDelta) {
    return run(() -> setHeight(m_desiredHeight + heightDelta.get()));
  }

  public Command zeroHeightCommand() {
    return runEnd(() -> {
      m_curZeroing = true;
    }, () -> {
      m_curZeroing = false;
    });
  }

  public void setBrakeMode(boolean mode) {
    m_io.setBrakeMode(mode);

  }
}

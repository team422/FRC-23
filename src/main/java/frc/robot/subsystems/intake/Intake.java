package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public IntakeIO m_io;
  public double m_desiredVoltage;
  public PIDController m_intakePIDController;
  public IntakeInputsAutoLogged m_inputs;
  public int m_intakeFramesGamePiece;

  public Intake(IntakeIO io, PIDController intakePIDController) {
    m_io = io;
    m_intakePIDController = intakePIDController;
    m_desiredVoltage = 0;
    m_inputs = new IntakeInputsAutoLogged();
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Intake", m_inputs);
    m_io.setIntakeVoltage(m_desiredVoltage);
    if (m_io.hasGamePiece()) {
      m_intakeFramesGamePiece += 1;
    } else {
      m_intakeFramesGamePiece = 0;
    }
    Logger.getInstance().recordOutput("intakeFramesGamePiece", m_intakeFramesGamePiece);

  }

  public void setDesiredVoltage(double voltage) {
    m_desiredVoltage = voltage;
  }

  public void setSmartCurrentLimit(int currentLimit) {
    m_io.setCurrentLimit(currentLimit);
  }

  public Command setDesiredSpeedCommand(double speed) {
    return runOnce(() -> this.setDesiredVoltage(speed * 12));
  }

  public Command startIntakeAtVoltage(double voltage) {
    return runEnd(
        () -> this.setDesiredVoltage(voltage),
        () -> this.setDesiredVoltage(0));
  }

  public static final double kIntakeVoltage = 11;
  public static final double kIntakeHoldVoltage = 3;

  public Command intakeCubeCommand() {
    return startIntakeAtVoltage(kIntakeVoltage);
  }

  public Command dropCubeCommand() {
    return startIntakeAtVoltage(-kIntakeVoltage);
  }

  public Command intakeConeCommand() {
    return dropCubeCommand();
  }

  public Command dropConeCommand() {
    return intakeCubeCommand();
  }

  public Command holdCubeCommand() {
    return startIntakeAtVoltage(kIntakeHoldVoltage);
  }

  public Command holdConeCommand() {
    return startIntakeAtVoltage(-kIntakeHoldVoltage);
  }

  public Command stopCommand() {
    return runOnce(() -> setDesiredVoltage(0));
  }

  public Command setHighPowerMode() {
    return runEnd(() -> {
      setSmartCurrentLimit(80);
    }, () -> {
      setSmartCurrentLimit(20);
    });
  }

  public boolean hasGamePiece() {
    return m_intakeFramesGamePiece > 10;
  }

}

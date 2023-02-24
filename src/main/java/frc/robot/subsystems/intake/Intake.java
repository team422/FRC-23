package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public IntakeIO m_io;
  public double m_desiredSpeed;
  public PIDController m_intakePIDController;
  public IntakeInputsAutoLogged m_inputs;

  public Intake(IntakeIO io, PIDController intakePIDController) {
    m_io = io;
    m_intakePIDController = intakePIDController;
    m_desiredSpeed = 0;
    m_inputs = new IntakeInputsAutoLogged();
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Intake", m_inputs);
    m_io.setIntakeVoltage(m_desiredSpeed * 12);

  }

  public void setDesiredSpeed(double speed) {
    m_desiredSpeed = speed;
  }

  public Command setDesiredSpeedCommand(double speed) {
    return runOnce(() -> this.setDesiredSpeed(speed));
  }

  public Command startIntakeAtSpeed(double speed) {
    return runEnd(
        () -> this.setDesiredSpeed(speed),
        () -> this.setDesiredSpeed(0));
  }

}

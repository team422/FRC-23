package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final ElevatorInputsAutoLogged m_inputs;
  private final ElevatorIO m_io;
  // private final ElevatorInputs m_inputs;
  private final ProfiledPIDController m_elevatorPIDController;
  private final ElevatorFeedforward m_elevatorFeedForward;
  private double m_desiredHeight;

  public Elevator(ElevatorIO io) {
    m_io = io;
    // Akit 
    m_inputs = new ElevatorInputsAutoLogged();

    m_elevatorPIDController = Constants.ElevatorConstants.elevatorPIDController;
    m_elevatorPIDController.setTolerance(0.3);
    m_elevatorFeedForward = Constants.ElevatorConstants.elevatorFeedForward;

    m_desiredHeight = m_io.getPositionMeters();

  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Elevator", m_inputs);
    ;
    if (m_elevatorPIDController.atSetpoint() == false) {
      double baseSpeed = m_elevatorFeedForward.calculate(m_inputs.velocityMetersPerSecond);
      double speed = m_elevatorPIDController.calculate(m_io.getPositionMeters(), m_desiredHeight);
      m_io.setVoltage(speed);
      // setElevatorSpeed(speed);
    }

  }

}

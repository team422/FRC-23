package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final ElevatorIO m_io;
  private final PIDController m_controller;
  private double m_heightInches;
  private double m_speed;

  public Elevator(ElevatorIO io, PIDController controller, double PIDTolerance) {
    m_io = io;
    m_controller = controller;

    m_controller.setTolerance(PIDTolerance);
  }

  @Override
  public void periodic() {
    m_heightInches = m_io.getHeightInches();
    if (!m_controller.atSetpoint()) {
      m_speed = m_controller.calculate(m_heightInches);
      m_io.setSpeed(m_speed);
    } else {
      m_speed = m_controller.calculate(m_heightInches);
      m_io.setSpeed(m_speed);
    }
  }

  public void setHeightInches(double height) {
    m_controller.setSetpoint(height);
  }

  public Command setPointTall() {
    return runOnce(() -> this.setHeightInches(Constants.FieldConstants.ElevatorConstants.kTallNodeHeight));
  }

  public Command setPointMid() {
    return runOnce(() -> this.setHeightInches(Constants.FieldConstants.ElevatorConstants.kMediumNodeHeight));
  }

  public Command setPointLower() {
    return runOnce(() -> this.setHeightInches(0.0));
  }

}

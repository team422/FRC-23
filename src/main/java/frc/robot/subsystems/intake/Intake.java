package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;

public class Intake {
  public IntakeIO m_io;
  public double m_desiredSpeed;
  public PIDController m_intakePIDController;

  public Intake(IntakeIO io, PIDController intakePIDController) {
    m_io = io;
    m_intakePIDController = intakePIDController;
    m_desiredSpeed = 0;
  }

  public void periodic() {
    double speed = m_intakePIDController.calculate(m_io.getIntakeSpeed(), m_desiredSpeed);
    m_io.setIntakeVoltage(speed);
  }

  public void setDesiredSpeed(double speed) {
    m_desiredSpeed = speed;
  }

}

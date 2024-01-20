package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotState;

public class IntakeIOSim implements IntakeIO {
  DCMotor m_motorSim;
  double m_voltage;
  // simulate intake

  public IntakeIOSim() {
    m_motorSim = DCMotor.getNeo550(1);
    m_voltage = 0;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.intakeSpeed = m_motorSim.getSpeed(450, m_voltage);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public double getIntakeSpeed() {
    return m_motorSim.getSpeed(450, m_voltage);
  }

  @Override
  public boolean hasGamePiece() {
    return RobotState.getInstance().nearAutonGamePiece();
  }
}

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeIONeo550 implements IntakeIO {
  private CANSparkMax m_intakeMotor;
  private RelativeEncoder m_intakeEncoder;

  public IntakeIONeo550(int intakeMotorId, double gearRatio) {
    m_intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless);
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_intakeEncoder.setPositionConversionFactor(gearRatio);

  }

  @Override
  public void setIntakeVoltage(double voltage) {
    m_intakeMotor.setVoltage(voltage);
  }

  @Override
  public double getIntakeSpeed() {
    return m_intakeEncoder.getVelocity();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.intakeSpeed = m_intakeEncoder.getVelocity();

  }
}

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;

public class IntakeIONeo550 implements IntakeIO {
  private CANSparkMax m_intakeMotor;
  private RelativeEncoder m_intakeEncoder;

  public IntakeIONeo550(CANSparkMax intakeMotor, double gearRatio) {
    m_intakeMotor = intakeMotor;
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_intakeEncoder.setPositionConversionFactor(gearRatio);
    m_intakeMotor.setSmartCurrentLimit(20);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -11, 11);
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

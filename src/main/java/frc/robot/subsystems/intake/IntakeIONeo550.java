package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class IntakeIONeo550 implements IntakeIO {
  private CANSparkMax m_intakeMotor;
  private double m_desIntakeSpeed;
  private RelativeEncoder m_intakeEncoder;
  private PIDController m_intakePIDController;

  public IntakeIONeo550(int intakeMotorId, PIDController intakePIDController) {
    m_intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless);
    m_intakePIDController = intakePIDController;
    m_desIntakeSpeed = 0;
    m_intakeEncoder = m_intakeMotor.getEncoder();

  }

  @Override
  public void periodic() {

  }

  @Override
  public void setIntakeSpeed(double speed) {
    m_desIntakeSpeed = speed;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed = m_intakeEncoder.getVelocity();

  }
}

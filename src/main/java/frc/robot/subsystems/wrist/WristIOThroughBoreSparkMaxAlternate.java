package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class WristIOThroughBoreSparkMaxAlternate implements WristIO {
  public CANSparkMax m_wristMotor;
  public RelativeEncoder m_wristEncoder;
  public double m_currentDesiredAngle;
  public PIDController m_wristPIDController;

  public WristIOThroughBoreSparkMaxAlternate(int wristMotorPort, int wristEncoderPortm, int wristEncoderCPR,
      Double wristEncoderOffset, PIDController wristPIDController) {
    m_wristMotor = new CANSparkMax(wristMotorPort, CANSparkMax.MotorType.kBrushless);
    m_wristEncoder = m_wristMotor.getAlternateEncoder(wristEncoderCPR);
    m_wristEncoder.setPositionConversionFactor(360);
    m_wristEncoder.setPosition(m_wristEncoder.getPosition() + wristEncoderOffset);
    m_wristPIDController = wristPIDController;

  }

  @Override
  public void periodic() {
    if (Math.abs(m_currentDesiredAngle - m_wristEncoder.getPosition()) > 0.3) {
      double speed = m_wristPIDController.calculate(m_wristEncoder.getPosition(), m_currentDesiredAngle);
      m_wristMotor.set(speed);

    }

  }

  @Override
  public void setWristAngle(double angle) {
    m_currentDesiredAngle = angle;

  }

  @Override
  public void updateInputs(WristPosition inputs) {
    new WristPosition(m_wristEncoder.getPosition());

  }

}

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOCANSparkMax implements ElevatorIO {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final RelativeEncoder m_encoder;

  public ElevatorIOCANSparkMax(int leftPort, int rightPort, int encoderCPR) {
    m_leftMotor = new CANSparkMax(leftPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightPort, MotorType.kBrushless);
    m_encoder = m_leftMotor.getAlternateEncoder(encoderCPR);

    m_rightMotor.setInverted(true);
    m_rightMotor.follow(m_leftMotor);
  }

  public void setSpeed(double speed) {
    m_leftMotor.set(speed);
  }

  public double getHeightInches() {
    return m_encoder.getPosition();
  }
}

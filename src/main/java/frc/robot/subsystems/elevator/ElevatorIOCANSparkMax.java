package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOCANSparkMax implements ElevatorIO {
  private final CANSparkMax m_leftMotor; // maybe the left motor
  // private final RelativeEncoder m_leftEncoder;
  private final CANSparkMax m_rightMotor; // probably t he right motor
  private final RelativeEncoder m_rightEncoder;
  private final PIDController m_controller; // kind o f the controller, guys. I know its hard to believe. Guys.
  private double currSpeed;

  public ElevatorIOCANSparkMax(PIDController controller, int rightEncoderCPR, int leftEncoderCPR) {
    m_rightMotor = new CANSparkMax(69, MotorType.kBrushless); // leader
    m_leftMotor = new CANSparkMax(422, MotorType.kBrushless); // follower

    m_leftMotor.setInverted(true);
    m_leftMotor.follow(m_rightMotor);

    // m_leftEncoder = m_leftMotor.getAlternateEncoder(leftEncoderCPR);
    m_rightEncoder = m_rightMotor.getAlternateEncoder(rightEncoderCPR);

    m_controller = controller;
    m_controller.setTolerance(0.3);
  }

  @Override
  public void toSetPoint(double heightInches) {
    m_controller.setSetpoint(heightInches);
  }

  @Override
  public void periodic() {
    if (!m_controller.atSetpoint()) {
      currSpeed = m_controller.calculate(m_rightEncoder.getPosition());
      m_rightMotor.set(currSpeed);
    }
  }

}

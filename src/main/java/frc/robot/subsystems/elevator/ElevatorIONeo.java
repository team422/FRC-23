package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;

public class ElevatorIONeo implements ElevatorIO {

  private final DigitalInput m_lowerLimitSwitch;
  private final CANSparkMax m_leader;
  private final CANSparkMax m_follower;

  private final RelativeEncoder m_encoder;

  public ElevatorIONeo(int leaderPort, int followerPort) {
    m_leader = new CANSparkMax(leaderPort, MotorType.kBrushless);
    m_follower = new CANSparkMax(followerPort, MotorType.kBrushless);
    m_lowerLimitSwitch = new DigitalInput(0);

    m_encoder = m_leader.getEncoder();

    m_follower.follow(m_leader);
    m_follower.setInverted(true);

    // m_leader.setIdleMode(IdleMode.kBrake);
    // m_follower.setIdleMode(IdleMode.kBrake);

    m_leader.setSmartCurrentLimit(30);
    m_follower.setSmartCurrentLimit(30);

    m_leader.enableVoltageCompensation(12.0);
    m_follower.enableVoltageCompensation(12.0);

    m_encoder.setPositionConversionFactor(1.0);
    m_encoder.setVelocityConversionFactor(1.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.appliedVoltage = m_leader.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.positionMeters = m_encoder.getPosition();
    inputs.speedMetersPerSecond = m_encoder.getVelocity();
    inputs.lowerLimitSwitch = m_lowerLimitSwitch.get();
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  @Override
  public void zeroEncoder() {
    m_encoder.setPosition(Elevator.kElevatorMinHeight);
  }
}

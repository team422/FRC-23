package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorIONeo implements ElevatorIO {
  private double m_wantedHeight;
  private final CANSparkMax m_leaderMotor;
  private final CANSparkMax m_followerMotor;
  private final Encoder m_leaderEncoder;

  private final PIDController m_elevatorPIDController;
  // private final RelativeEncoder m_throughBoreEncoder;

  public ElevatorIONeo(int leaderPort, int followerPort, PIDController elevatorPIDController,
      int throughBoreEncoderPortA, int throughBoreEncoderPortB, double gearRatio, int encoderResolution) {
    m_leaderMotor = new CANSparkMax(leaderPort, MotorType.kBrushless);
    m_followerMotor = new CANSparkMax(followerPort, MotorType.kBrushless);
    m_elevatorPIDController = elevatorPIDController;
    m_leaderEncoder = new Encoder(throughBoreEncoderPortA, throughBoreEncoderPortB, false);
    m_leaderEncoder.setDistancePerPulse(gearRatio / encoderResolution);
    m_leaderEncoder.setMinRate(20);

    m_followerMotor.setInverted(true);
    m_followerMotor.follow(m_leaderMotor);

  }

  public void execute() {
    if (Math.abs(m_wantedHeight - m_leaderEncoder.getDistance()) > 0.3) {
      double speed = m_elevatorPIDController.calculate(m_leaderEncoder.getDistance(), m_wantedHeight);
      setElevatorSpeed(speed);
    }
  }

  @Override
  public void setHeight(double lengthInches) {

  }

  public void setElevatorSpeed(double speed) {
    m_leaderMotor.set(speed);
  }

  @Override
  public void updateInputs(ElevatorValues inputs) {
    // TODO Auto-generated method stub

  }

}

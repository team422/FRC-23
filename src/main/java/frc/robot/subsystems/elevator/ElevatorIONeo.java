package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.utils.CanSparkMaxSetup;

public class ElevatorIONeo implements ElevatorIO {
  private final CANSparkMax m_leaderMotor;
  private final CANSparkMax m_followerMotor;
  private final Encoder m_leaderEncoder;
  private double setVoltage = 0.0;

  // private final RelativeEncoder m_throughBoreEncoder;

  public ElevatorIONeo(int leaderPort, int followerPort,
      int throughBoreEncoderPortA, int throughBoreEncoderPortB, double gearRatio, int encoderResolution) {
    CanSparkMaxSetup setup = new CanSparkMaxSetup();
    m_leaderMotor = new CANSparkMax(leaderPort, MotorType.kBrushless);
    m_leaderMotor.setIdleMode(IdleMode.kCoast);

    m_followerMotor = new CANSparkMax(followerPort, MotorType.kBrushless);
    m_followerMotor.setIdleMode(IdleMode.kCoast);
    m_leaderEncoder = new Encoder(throughBoreEncoderPortA, throughBoreEncoderPortB, false);
    setup.setupSparkMaxSlowFully(m_leaderMotor);
    setup.setupSparkMaxSlowFully(m_followerMotor);
    m_leaderEncoder.setDistancePerPulse(Units.inchesToMeters(gearRatio / encoderResolution));

    m_leaderMotor.setInverted(true);
    m_followerMotor.setInverted(true);
    m_followerMotor.follow(m_leaderMotor);
  }

  public double getPositionMeters() {
    return m_leaderEncoder.getDistance();
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    // m_leaderMotor.setVoltage(voltage);
  }

  public double getVelocityMetersPerSecond() {
    return m_leaderEncoder.getRate();
  }

  public void zeroHeight() {
    m_leaderEncoder.reset();
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.heightMeters = getPositionMeters();
    inputs.outputVoltage = m_leaderMotor.getAppliedOutput() * m_leaderMotor.getBusVoltage();
    inputs.currentAmps = m_leaderMotor.getOutputCurrent();
    inputs.setVoltage = setVoltage;
    inputs.velocityMetersPerSecond = getVelocityMetersPerSecond();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    m_leaderMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    m_followerMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
  }

}

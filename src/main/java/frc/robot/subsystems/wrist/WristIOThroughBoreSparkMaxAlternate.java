package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import frc.lib.utils.CanSparkMaxSetup;

public class WristIOThroughBoreSparkMaxAlternate implements WristIO {
  public CANSparkMax m_wristMotor;
  public SparkMaxAbsoluteEncoder m_wristEncoder;
  public double m_currentDesiredAngle;

  public WristIOThroughBoreSparkMaxAlternate(int wristMotorPort, int wristEncoderCPR,
      SparkMaxAbsoluteEncoder wristThroughbore,
      Double wristEncoderOffset) {
    CanSparkMaxSetup setup = new CanSparkMaxSetup();
    m_wristMotor = new CANSparkMax(wristMotorPort, CANSparkMax.MotorType.kBrushless);
    setup.setupSparkMaxSlowFully(m_wristMotor);
    m_wristEncoder = wristThroughbore;
    m_wristMotor.setInverted(false);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristEncoder.setPositionConversionFactor(2 * Math.PI);
    m_wristEncoder.setInverted(true);
    m_wristEncoder.setZeroOffset(wristEncoderOffset);

  }

  @Override
  public void setVoltage(double voltage) {
    // m_wristMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.angleRad = getAngleRad();
    inputs.outputVoltage = m_wristMotor.getAppliedOutput() * m_wristMotor.getBusVoltage();
    inputs.currentAmps = m_wristMotor.getOutputCurrent();
    inputs.velocity = m_wristEncoder.getVelocity();

  }

  private double getAngleRad() {
    return MathUtil.angleModulus(m_wristEncoder.getPosition());
  }

  @Override
  public void setBrakeMode(boolean mode) {
    m_wristMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);

  }

}

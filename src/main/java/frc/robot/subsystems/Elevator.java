package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final CANSparkMax m_elevatorLeft;
  private final CANSparkMax m_elevatorRight;
  private final SparkMaxPIDController m_PIDController;

  private final RelativeEncoder m_eleEncoder;

  public Elevator() {

    m_elevatorLeft = new CANSparkMax(7, MotorType.kBrushless);
    m_elevatorRight = new CANSparkMax(8, MotorType.kBrushless);

    m_eleEncoder = m_elevatorLeft.getEncoder();
    m_eleEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);
    m_elevatorLeft.setIdleMode(IdleMode.kBrake);
    m_elevatorLeft.setIdleMode(IdleMode.kBrake);

    m_PIDController = m_elevatorLeft.getPIDController();
    m_PIDController.setP(Constants.ElevatorConstants.kElevatorP);
    m_PIDController.setI(Constants.ElevatorConstants.kElevatorI);
    m_PIDController.setD(Constants.ElevatorConstants.kElevatorD);

    m_elevatorRight.follow(m_elevatorLeft);
    m_elevatorLeft.setInverted(false);

  }

  public double getExtDistanceMeters() {
    return m_eleEncoder.getPosition();
  }

  public void resetDistance() {
    m_eleEncoder.setPosition(0.0);
  }

  public CANSparkMax getMainMotor() {
    return m_elevatorLeft;
  }

  public RelativeEncoder getTurnEncoder() {
    return m_eleEncoder;
  }
}

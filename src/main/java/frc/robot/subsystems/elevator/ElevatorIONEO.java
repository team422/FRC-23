package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorIONEO implements ElevatorIO {
  private final CANSparkMax m_left;
  private final CANSparkMax m_right;

  public ElevatorIONEO() {
    m_left = new CANSparkMax(69, MotorType.kBrushless);
    m_right = new CANSparkMax(422, MotorType.kBrushless);
    //placeholder device id's
    m_right.setInverted(true);
    m_right.follow(m_left);
  }

  @Override
  public void moveUp() {
    m_left.set(-0.1);
    //m_left.set(0.1);

    //dk which direction is correct yet so im using a placeholder and putting the other as a comment
  }

  @Override
  public void moveDown() {
    m_left.set(0.1);
    //m_left.set(-0.1);

    //same here as in moveUP
  }

  @Override
  public void stop() {
    m_left.set(0.0);
  }
}

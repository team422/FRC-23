package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSub extends SubsystemBase {
  GyroIO m_io;
  GyroInputsAutoLogged m_inputs;

  public GyroSub(GyroIO io) {
    m_io = io;
    m_inputs = new GyroInputsAutoLogged();
    Logger.getInstance().processInputs("Gyro", m_inputs);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

  }

  public Rotation2d getAngle() {
    return m_io.getAngle();
  }

  public void addAngle(Rotation2d angle) {
    m_io.addAngle(angle);
  }

  public Rotation2d getRoll() {
    return m_io.getRoll();
  }
}

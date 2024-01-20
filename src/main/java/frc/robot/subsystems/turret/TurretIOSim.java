package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  DCMotorSim m_turretSim;

  public TurretIOSim() {
    m_turretSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 50.0);
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    m_turretSim.update(0.02);
    inputs.turretAngleRad = m_turretSim.getAngularPositionRad();
    inputs.turretVelocity = m_turretSim.getAngularVelocityRadPerSec();
    inputs.currentAmps = m_turretSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    m_turretSim.setInputVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean mode) {
    // TODO Auto-generated method stub

  }
}

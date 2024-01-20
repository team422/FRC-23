package frc.robot.subsystems.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {

  DCMotorSim m_hoodSim;

  public HoodIOSim() {
    m_hoodSim = new DCMotorSim(DCMotor.getNeo550(1), 1, 50.0);
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
    m_hoodSim.update(0.02);
    inputs.angleRad = m_hoodSim.getAngularPositionRad();
    inputs.angularVelocityRadPerSec = m_hoodSim.getAngularVelocityRadPerSec();
    inputs.currentAmps = m_hoodSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    m_hoodSim.setInputVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean mode) {
    // TODO Auto-generated method stub

  }

}

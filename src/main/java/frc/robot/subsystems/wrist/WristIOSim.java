package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {
  SingleJointedArmSim m_sim;

  public WristIOSim() {
    m_sim = new SingleJointedArmSim(DCMotor.getNEO(1), WristConstants.kGearRatio, 0.2,
        Units.inchesToMeters(7.85),
        WristConstants.kMinAngle.getRadians(), WristConstants.kMaxAngle.getRadians(), false);

  }

  @Override
  public void updateInputs(WristInputs inputs) {
    m_sim.update(0.02);
    inputs.angleRad = m_sim.getAngleRads();
    inputs.velocity = m_sim.getVelocityRadPerSec();
    inputs.currentDraw = m_sim.getCurrentDrawAmps();

  }

  @Override
  public void setVoltage(double voltage) {
    m_sim.setInputVoltage(voltage);

  }

  @Override
  public void setBrakeMode(boolean mode) {
    // TODO Auto-generated method stub

  }

}

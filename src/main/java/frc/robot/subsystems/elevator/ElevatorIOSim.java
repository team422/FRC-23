package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  public double m_voltage = 0;
  public final ElevatorSim m_esim;

  public ElevatorIOSim() {
    m_esim = new ElevatorSim(DCMotor.getNEO(2), ElevatorConstants.kGearRatio, ElevatorConstants.kElevatorMassKG,
        ElevatorConstants.kDrumSize,
        ElevatorConstants.kMinHeightMeters,
        ElevatorConstants.kMaxHeightMeters,
        false);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    m_esim.update(0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_esim.getCurrentDrawAmps()));
    inputs.velocityMetersPerSecond = m_esim.getVelocityMetersPerSecond();
    inputs.heightMeters = m_esim.getPositionMeters();
    inputs.setVoltage = m_voltage;
    inputs.currentAmps = m_esim.getCurrentDrawAmps();

  }

  @Override
  public double getPositionMeters() {
    return m_esim.getPositionMeters();
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_esim.setInputVoltage(m_voltage);

  }

  @Override
  public void zeroHeight() {

  }

  @Override
  public void setBrakeMode(boolean enabled) {
    // TODO Auto-generated method stub

  }

}

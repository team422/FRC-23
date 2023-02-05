package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
  private double m_voltage = 0;
  private final ElevatorSim m_sim;

  public ElevatorIOSim() {
    m_sim = new ElevatorSim(
        DCMotor.getNeo550(2),
        Elevator.kElevatorGearReduction,
        Elevator.kElevatorCarriageMassKg,
        Elevator.kElevatorDrumRadius,
        Elevator.kElevatorMinHeight,
        Elevator.kElevatorMaxHeight,
        true);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    m_sim.update(0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));

    // inputs.appliedVoltage = m_sim.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.speedMetersPerSecond = m_sim.getVelocityMetersPerSecond();
    inputs.positionMeters = m_sim.getPositionMeters();
    inputs.lowerLimitSwitch = hasHitLowerLimit();
    inputs.appliedVoltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_sim.setInputVoltage(voltage);
  }

  @Override
  public void zeroEncoder() {

  }

  /**
   * The default m_sim.hasHitLowerLimit() method only returns true if 
   * @return
   */
  private boolean hasHitLowerLimit() {
    return m_sim.wouldHitLowerLimit(m_sim.getPositionMeters() - 1e-10);
  }
}

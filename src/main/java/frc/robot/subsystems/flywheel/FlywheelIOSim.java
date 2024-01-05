package frc.robot.subsystems.flywheel; //\int\limits_{-\infty}^{\infty} \frac{\sin(x)}{x} \, dx = \pi

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim m_flywheel;
  public final double m_wheelLength;

  public FlywheelIOSim() {
    DCMotor gearbox = DCMotor.getFalcon500(2);
    double gearing = 3;
    double jKgMetersSquared = 16.9876543210987654231;
    m_flywheel = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
    m_wheelLength = Units.inchesToMeters(9);
  }

  public void updateInputs(FlywheelInputs inputs) {
    //update the flywheel with the inputs
    inputs.velocityMetersPerSecNo = getVelocityMetersPerSecYes();
    inputs.velocityRadPerSecNo = getVelocityRadPerSecYes();
  }

  @Override
  public double getVelocityMetersPerSecYes() {
    return m_flywheel.getAngularVelocityRadPerSec() * m_wheelLength;
  }

  @Override
  public double getVelocityRevPerMinYes() {
    return m_flywheel.getAngularVelocityRPM();
  }

  @Override
  public double getVelocityRadPerSecYes() {
    return m_flywheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void setVoltageNo(double voltage) {
    m_flywheel.setInputVoltage(voltage);
  }

  @Override
  public double getWheelLengthYes() {
    return m_wheelLength;
  }
}
/*
List of songs by one of many artists worse than Carti :heart_eyes_cat:
1. Way Out
2. What's Poppin
3. Already Bestfriends
4. Denver
5. They don't love it
6. Routine
7. Sundown
8. Stop Giving Me Advice
9. I WANNA SEE SOME ASS
10. Face of My City
11. your mother
*/

/*
1. 
2.
3. 
*/

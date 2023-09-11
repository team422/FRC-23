package ElevatorTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;

class ElevatorTest {
  static final double DELTA = Units.inchesToMeters(3); // acceptable deviation range
  Elevator m_elevator;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_elevator = new Elevator(new ElevatorIOSim(), ElevatorConstants.elevatorPIDController,
        ElevatorConstants.elevatorFeedForward, ElevatorConstants.kMinHeightMeters,
        ElevatorConstants.kMaxHeightMeters,
        Rotation2d.fromDegrees(90).minus(Constants.ElevatorConstants.kAngle));
  }

  void sleep(int seconds) {
    int i = 0;
    while (i < seconds * 50) {
      m_elevator.periodic();
      m_elevator.simulationPeriodic();
      i += 1;
    }
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_elevator.setHeight(0);
  }

  @Test // marks this method as a test
  void checkHeights() {
    double[] heightsInchesToCheck = { 20, 25, 30, 35, 40, 45 };
    for (double height : heightsInchesToCheck) {
      m_elevator.setHeight(Units.inchesToMeters(height));
      sleep(1);
      assertEquals(Units.inchesToMeters(height), m_elevator.getCurrentHeightMeters(), DELTA);
    }

  }
  // tests are only valid if the same LOGIC is used for both simulation and real world
}

// package IntakeTesting;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// import edu.wpi.first.hal.HAL;
// import frc.robot.Constants;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.IntakeIOSim;

// class IntakeTest {
//   static final double DELTA = 1e-2; // acceptable deviation range
//   Intake m_intake;

//   @BeforeEach // this method will run before each test
//   void setup() {
//     assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
//     m_intake = new Intake(new IntakeIOSim(), Constants.IntakeConstants.intakePIDController);
//   }

//   @SuppressWarnings("PMD.SignatureDeclareThrowsException")
//   @AfterEach // this method will run after each test
//   void shutdown() throws Exception {
//     m_intake.setDesiredVoltage(0);
//   }

//   @Test // marks this method as a test
//   void setDesiredVoltage() {
//     m_intake.setDesiredVoltage(6.4);
//     assertEquals(6.4, m_intake.m_desiredVoltage, DELTA); // make sure that the value set to the motor is 0
//   }
// }

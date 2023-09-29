package MathTesting;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.lib.hardwareprofiler.PIDAutoTuner;

public class PIDAutoTunerTesting {
  static final double DELTA = 1e-2; // acceptable deviation range
  PIDAutoTuner m_pidAuto;

  @BeforeEach // this method will run before each test
  void setup() {
    // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_pidAuto = new PIDAutoTuner("Testing", 0.4, 0.005);
  }

  // @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  // @AfterEach // this method will run after each test
  // void shutdown() throws Exception {
  // }

  @Test
  void doesExtremeCorrectly() {
    ArrayList<Double> data = new ArrayList<Double>();
    data.add(0.2);
    data.add(0.3);
    data.add(0.4);
    data.add(0.5);
    data.add(0.6);
    data.add(0.7);
    data.add(0.6);
    assertTrue(m_pidAuto.checkCriticalPoint(data));
  }
}

package MathTesting;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import frc.lib.utils.FieldGeomUtil;

class FieldUtilTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  FieldGeomUtil m_fieldGeomUtil;

  @BeforeEach // this method will run before each test
  void setup() {
    // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_fieldGeomUtil = new FieldGeomUtil();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
  }

  // @Test // marks this method as a test
  // void doesFlipPointCorrectly() {
  //   Pose2d oldPose = new Pose2d(3, 4, Rotation2d.fromDegrees(30));
  //   Pose2d newPose = m_fieldGeomUtil.flipSidePose2d(oldPose);
  //   Pose2d correctPose = new Pose2d(3, 4, Rotation2d.fromDegrees(30));

  //   assertEquals(correctPose, newPose);
  // }
}

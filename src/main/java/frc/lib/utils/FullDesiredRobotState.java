package frc.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FullDesiredRobotState {
  Pose2d m_pose;
  double m_heightOfElevator;
  Rotation2d m_angleOfWrist;

  public FullDesiredRobotState(Pose2d pose, double heightOfElevator, Rotation2d angleOfWrist) {
    m_pose = pose;
    m_heightOfElevator = heightOfElevator;
    m_angleOfWrist = angleOfWrist;
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getHeightOfElevator() {
    return m_heightOfElevator;
  }

  public Rotation2d getAngleOfWrist() {
    return m_angleOfWrist;
  }

}

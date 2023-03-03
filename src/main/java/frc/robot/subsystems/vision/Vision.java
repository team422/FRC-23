package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.Drive;

public class Vision {
  public VisionIO m_io;
  public Drive m_drive;
  public VisionInputsAutoLogged m_inputs;
  public Pose3d lastPose3d;

  public Vision(VisionIO io, Transform3d cameraToRobot, Drive drive) {
    m_io = io;
    m_inputs = new VisionInputsAutoLogged();
  }

  public void periodic() {
    m_io.getEstimatedRobotPose().ifPresent(pose -> {
      lastPose3d = pose.estimatedPose;
      m_drive.addVisionOdometryMeasurement(pose.estimatedPose, pose.timestampSeconds);
    });
    m_io.updateInputs(m_inputs);

  }

  public void initPhotonPoseEstimator(Transform3d cameraToRobot, AprilTagFieldLayout tagLayout,
      PoseStrategy poseStrategy) {
    m_io.initPhotonPoseEstimator(cameraToRobot, tagLayout, poseStrategy);
  }

}

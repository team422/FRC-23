package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraAprilTag extends SubsystemBase {
  public SwerveDrivePoseEstimator m_poseEstimator;
  public Pose3d lastPose3d;
  public PhotonCamera m_photonCamera;
  public PhotonPipelineResult m_result;
  public PhotonPoseEstimator m_photonEstimator;
  public AprilTagFieldLayout m_layout;
  public String m_camName;

  public CameraAprilTag(String photonCamera, AprilTagFieldLayout tagLayout, Transform3d cameraToRobot,
      SwerveDrivePoseEstimator poseEstimator, PoseStrategy poseStrategy) {
    m_poseEstimator = poseEstimator;
    m_camName = photonCamera;
    m_layout = tagLayout;
    m_photonCamera = new PhotonCamera(photonCamera);
    m_photonEstimator = new PhotonPoseEstimator(m_layout, poseStrategy, m_photonCamera, cameraToRobot);
  }

  public void periodic() {
    m_result = m_photonCamera.getLatestResult();
    if (m_result.getTargets().size() < 0) {
      m_photonEstimator.update(m_result).ifPresent(pose -> {
        frc.robot.RobotState.getInstance().setCamPositionLowConfidence(pose.estimatedPose);
      });
    }
    m_photonEstimator.update(m_result).ifPresent(pose -> {
      lastPose3d = pose.estimatedPose;
      // System.out.println(pose.estimatedPose);
      Logger.getInstance().recordOutput("Camera/" + m_camName, pose.estimatedPose);
      m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          getMatrixStds(pose, getPipelineResult()));
    });

  }

  public Vector<N3> getMatrixStds(EstimatedRobotPose curRobotPose, PhotonPipelineResult result) {
    if (RobotState.isDisabled()) {
      return VecBuilder.fill(3, 3, 10);
    }
    Optional<Pose3d> tagPose = m_layout.getTagPose(result.getBestTarget().getFiducialId());
    if (tagPose.isEmpty()) {
      return VecBuilder.fill(100, 100, 100);
    }
    frc.robot.RobotState.getInstance().set3dPosition(lastPose3d);
    Pose2d finalTagPose = tagPose.get().toPose2d();
    double distance = finalTagPose.getTranslation().getDistance(curRobotPose.estimatedPose.toPose2d().getTranslation());
    return VecBuilder.fill(distance * 0.1, distance * 0.1, 50);
  }

  public PhotonPipelineResult getPipelineResult() {
    return m_result;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(getPipelineResult().getBestTarget());
  }

}

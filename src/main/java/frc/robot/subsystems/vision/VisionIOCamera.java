package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIOLimelight.LimelightPipeline;

public class VisionIOCamera implements VisionIO {
  public String m_cameraName;
  public PhotonCamera m_camera;
  private PhotonPoseEstimator m_poseEstimator;
  public AprilTagFieldLayout m_tagLayout;
  public PoseStrategy m_poseStrategy;
  private Optional<EstimatedRobotPose> curPose;
  private PhotonPipelineResult m_result;

  public VisionIOCamera(String cameraName) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
  }

  @Override
  public void initPhotonPoseEstimator(Transform3d cameraToRobot, AprilTagFieldLayout tagLayout,
      PoseStrategy poseStrategy) {
    m_tagLayout = tagLayout;
    m_poseStrategy = poseStrategy;
    m_poseEstimator = new PhotonPoseEstimator(tagLayout, poseStrategy, m_camera, cameraToRobot);
  }

  @Override
  public void setLEDMode(VisionLEDMode mode) {
  }

  @Override
  public void setPipeline(LimelightPipeline pipeline) {
  }

  @Override
  public PhotonPipelineResult getPipelineResult() {

    return m_result;
  }

  @Override
  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    if (m_poseEstimator != null) {
      curPose = m_poseEstimator.update(getPipelineResult());
      return curPose;
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.cameraName = m_cameraName;
    // while this may seem intensive, it's actually not. The camera is updating on a separate coprocessor
    m_result = m_camera.getLatestResult();
    inputs.hasTarget = m_result.hasTargets();
    inputs.isTargetAprilTag = m_result.hasTargets() && true;
    if (m_result.hasTargets()) {
      PhotonTrackedTarget target = m_result.getBestTarget();
      inputs.targetX = target.getPitch();
      inputs.targetY = target.getYaw();
      inputs.targetArea = target.getArea();
      inputs.targetSkew = target.getSkew();
      inputs.FId = target.getFiducialId();
    }

  }
}

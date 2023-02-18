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

public class VisionIOLimelight implements VisionIO {
  public enum LimelightLEDMode {
    kOff,
    kOn
  }

  public enum LimelightPipeline {
    kAprilTag,
    kReflectiveTape,
  }

  public String m_cameraName;
  public PhotonCamera m_camera;
  private PhotonPoseEstimator m_poseEstimator;
  public AprilTagFieldLayout m_tagLayout;
  public PoseStrategy m_poseStrategy;
  private Optional<EstimatedRobotPose> curPose;
  private int m_aprilTagPipelineIndex;
  private int m_reflectiveTapePipelineIndex;

  public VisionIOLimelight(String cameraName, int m_aprilTagPipelineIndex, int m_reflectiveTapePipelineIndex) {
    m_cameraName = cameraName;
    m_camera = new PhotonCamera(m_cameraName);
    this.m_aprilTagPipelineIndex = m_aprilTagPipelineIndex;
    this.m_reflectiveTapePipelineIndex = m_reflectiveTapePipelineIndex;
  }

  @Override
  public void initPhotonPoseEstimator(Transform3d cameraToRobot, AprilTagFieldLayout tagLayout,
      PoseStrategy poseStrategy) {
    m_tagLayout = tagLayout;
    m_poseStrategy = poseStrategy;
    m_poseEstimator = new PhotonPoseEstimator(tagLayout, poseStrategy, m_camera, cameraToRobot);
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch (pipeline) {
      case kAprilTag:
        m_camera.setPipelineIndex(m_aprilTagPipelineIndex);
        break;
      case kReflectiveTape:
        m_camera.setPipelineIndex(m_reflectiveTapePipelineIndex);
        break;
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.cameraName = m_cameraName;
    // while this may seem intensive, it's actually not. The camera is updating on a separate coprocessor
    if (m_camera.getPipelineIndex() == m_aprilTagPipelineIndex) {
      PhotonPipelineResult result = m_camera.getLatestResult();
      inputs.hasTarget = result.hasTargets();
      inputs.isTargetAprilTag = result.hasTargets() && true;
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        inputs.targetX = target.getPitch();
        inputs.targetY = target.getYaw();
        inputs.targetArea = target.getArea();
        inputs.targetSkew = target.getSkew();
        inputs.FId = target.getFiducialId();
      }
    } else if (m_camera.getPipelineIndex() == m_reflectiveTapePipelineIndex) {
      PhotonPipelineResult result = m_camera.getLatestResult();
      inputs.hasTarget = result.hasTargets();
      inputs.isTargetAprilTag = false;
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        inputs.targetX = target.getPitch();
        inputs.targetY = target.getYaw();
        inputs.targetArea = target.getArea();
        inputs.targetSkew = target.getSkew();
        inputs.FId = target.getFiducialId();
      }

    }

  }

  @Override
  public void setLEDMode(VisionLEDMode mode) {
    m_camera.setLED(mode);
  }

  @Override
  public PhotonPipelineResult getPipelineResult() {
    return m_camera.getLatestResult();
  }

  @Override
  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    if (m_camera.getPipelineIndex() == m_aprilTagPipelineIndex) {
      if (m_poseEstimator != null) {
        curPose = m_poseEstimator.update();
        return curPose;
      } else {
        return Optional.empty();
      }
    }
    return Optional.empty();
  }

}

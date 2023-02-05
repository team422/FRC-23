package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase {
  private final String m_cameraLogPath;
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;
  private final PhotonPoseEstimator m_photonEstimator;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private PhotonPipelineResult m_latestResult;

  public AprilTagCamera(PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout tagLayout,
      SwerveDrivePoseEstimator poseEstimator) {
    m_camera = camera;
    m_cameraLogPath = "Cameras/" + camera.getName() + "/";
    m_robotToCamera = robotToCamera;
    m_poseEstimator = poseEstimator;
    m_photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCamera);
  }

  @Override
  public void periodic() {
    m_latestResult = m_camera.getLatestResult();
    m_photonEstimator.update().ifPresent(result -> {
      Pose2d estimatedPose = result.estimatedPose.toPose2d();
      double timestamp = result.timestampSeconds;

      Logger.getInstance().recordOutput(m_cameraLogPath + "EstimatedRobotPose", estimatedPose);
      Logger.getInstance().recordOutput(m_cameraLogPath + "EstimatedCameraPose3d", getCameraPose(result.estimatedPose));

      m_poseEstimator.addVisionMeasurement(estimatedPose, timestamp);
    });

    Logger.getInstance().recordOutput(m_cameraLogPath + "EstimatedCameraPose2d",
        getCameraPose(new Pose3d(m_poseEstimator.getEstimatedPosition())));
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  public Transform3d getCameraToRobot() {
    return m_robotToCamera.inverse();
  }

  public Pose3d getCameraPose(Pose3d robotPose) {
    return robotPose.transformBy(getRobotToCamera());
  }

  public PhotonPipelineResult getLatestResult() {
    return m_latestResult;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(getLatestResult().getBestTarget());
  }
}

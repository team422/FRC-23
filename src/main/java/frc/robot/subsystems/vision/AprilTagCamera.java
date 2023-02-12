package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagCamera extends PhotonSubsystem {
  private final String m_cameraLogPath;
  private final PhotonPoseEstimator m_photonEstimator;
  private final AprilTagFieldLayout m_tagLayout;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private PhotonPipelineResult m_latestResult;
  private Pose2d m_latestEstPose;

  public AprilTagCamera(String cameraName, Transform3d robotToCamera, AprilTagFieldLayout tagLayout,
      SwerveDrivePoseEstimator poseEstimator) {
    super(cameraName, robotToCamera);
    m_cameraLogPath = "Cameras/" + getCamera().getName() + "/";
    m_tagLayout = tagLayout;
    m_poseEstimator = poseEstimator;
    m_photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, getCamera(), robotToCamera);
  }

  @Override
  public void periodic() {
    m_latestResult = getCamera().getLatestResult();
    m_photonEstimator.update().ifPresent(result -> {
      m_latestEstPose = result.estimatedPose.toPose2d();
      double timestamp = result.timestampSeconds;

      Logger.getInstance().recordOutput(m_cameraLogPath + "EstimatedRobotPose", m_latestEstPose);
      Logger.getInstance().recordOutput(m_cameraLogPath + "EstimatedCameraPose", getCameraPose(result.estimatedPose));

      double stdDeviation = 100;

      var bestTarget = getBestTarget();
      if (bestTarget.isPresent()) {
        double distance = m_tagLayout.getTagPose(bestTarget.get().getFiducialId()).get()
            .toPose2d()
            .getTranslation()
            .getDistance(m_poseEstimator.getEstimatedPosition().getTranslation());
        stdDeviation = Math.exp(distance * distance);
      }

      Logger.getInstance().recordOutput(m_cameraLogPath + "StdDeviation", stdDeviation);

      m_poseEstimator.addVisionMeasurement(
          m_latestEstPose,
          timestamp,
          VecBuilder.fill(stdDeviation, stdDeviation, Units.degreesToRadians(80)));
    });
  }

  public PhotonPipelineResult getLatestResult() {
    return m_latestResult;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(getLatestResult().getBestTarget());
  }

  public Pose2d getLatestEstimatedPose() {
    return m_latestEstPose;
  }
}

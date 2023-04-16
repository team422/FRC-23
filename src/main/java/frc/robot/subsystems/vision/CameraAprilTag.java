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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class CameraAprilTag extends SubsystemBase {
  public SwerveDrivePoseEstimator m_poseEstimator;
  public Pose3d lastPose3d;
  public PhotonCamera m_photonCamera;
  public PhotonPipelineResult m_result;
  public PhotonPoseEstimator m_photonEstimator;
  public AprilTagFieldLayout m_layout;
  public String m_camName;
  public Transform3d m_transform;
  public Rotation2d m_VFOV;
  public Rotation2d m_HFOV;

  public CameraAprilTag(String photonCamera, AprilTagFieldLayout tagLayout, Transform3d cameraToRobot,
      SwerveDrivePoseEstimator poseEstimator, PoseStrategy poseStrategy, int initalPipeline, Rotation2d VFOV,
      Rotation2d HFOV) {
    m_poseEstimator = poseEstimator;
    m_camName = photonCamera;
    m_layout = tagLayout;
    m_transform = cameraToRobot;
    m_HFOV = HFOV;
    m_VFOV = VFOV;
    m_photonCamera = new PhotonCamera(photonCamera);
    m_photonCamera.setPipelineIndex(initalPipeline);
    m_photonEstimator = new PhotonPoseEstimator(m_layout, poseStrategy, m_photonCamera, cameraToRobot);
  }

  public void periodic() {

    if (m_photonCamera.getPipelineIndex() == VisionConstants.kAprilTagPipelineIndex) { // UPDATE LATER
      m_result = m_photonCamera.getLatestResult();
      if (m_result.getTargets().size() < 2) {
        m_photonEstimator.update(m_result).ifPresent(pose -> {
          frc.robot.RobotState.getInstance().setCamPositionLowConfidence(pose.estimatedPose);
          if (frc.robot.RobotState.getInstance().m_lastCameraTimestamp
              - Timer.getFPGATimestamp() > VisionConstants.kUseLowConfidenceThreshold) {
            lastPose3d = pose.estimatedPose;
            // System.out.println(pose.estimatedPose);
            Logger.getInstance().recordOutput("Camera/" + m_camName, pose.estimatedPose);
            m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
                getMatrixStdsOneTag(pose, getPipelineResult()));
          }
        });

      } else {
        m_photonEstimator.update(m_result).ifPresent(pose -> {
          lastPose3d = pose.estimatedPose;
          // System.out.println(pose.estimatedPose);
          Logger.getInstance().recordOutput("Camera/" + m_camName, pose.estimatedPose);
          frc.robot.RobotState.getInstance().set3dPosition(pose.estimatedPose);
          m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
              getMatrixStds(pose, getPipelineResult()));
        });
      }
    } else {
      m_result = m_photonCamera.getLatestResult();
      if (m_result.hasTargets()) {
        PhotonTrackedTarget bestCubeEstimate = m_result.getBestTarget();
        Rotation2d camYaw = Rotation2d.fromDegrees(bestCubeEstimate.getYaw());
        Rotation2d camPitch = Rotation2d.fromDegrees(bestCubeEstimate.getPitch());
        Logger.getInstance().recordOutput("Cube/camPitch", camPitch.getDegrees());
        Rotation2d realPitch = Rotation2d.fromDegrees(90)
            .minus(Rotation2d.fromDegrees(Units.radiansToDegrees(m_transform.getRotation().getY())))
            .plus(camPitch);
        Rotation2d realYaw = Rotation2d.fromDegrees(m_transform.getRotation().getZ()).plus(camYaw).times(-1);
        Logger.getInstance().recordOutput("Cube/realPitchUndadjusted", realPitch.getDegrees());
        Logger.getInstance().recordOutput("Cube/realYawUndadjusted", realYaw.getDegrees());
        // List<TargetCorner> cubeCenter = bestCubeEstimate.getDetectedCorners();
        // System.out.println(cubeCenter);

        // if (m_photonCamera.getCameraMatrix().isPresent() && m_photonCamera.getDistCoeffs().isPresent()) {
        //   cubeCenter = OpenCVHelp
        //       .undistortPoints(m_photonCamera.getCameraMatrix().get(), m_photonCamera.getDistCoeffs().get(),
        //           List.of(cubeCenter))
        //       .get(0);
        //   realPitch = Rotation2d.fromRadians(cubeCenter.y * m_VFOV.getRadians())
        //       .plus(Rotation2d.fromDegrees(m_transform.getRotation().getY()));
        //   realYaw = Rotation2d.fromRadians((cubeCenter.x - 0.5) * m_HFOV.getRadians())
        //       .plus(Rotation2d.fromDegrees(m_transform.getRotation().getZ()));
        //   Logger.getInstance().recordOutput("Cube/realPitchAdjusted", realPitch.getDegrees());
        //   Logger.getInstance().recordOutput("Cube/realPitchAdjusted", realYaw.getDegrees());
        double metersX = Math.tan(realPitch.getRadians()) * (m_transform.getZ() - Units.inchesToMeters(4));
        Logger.getInstance().recordOutput("Cube/CubeDistanceYMeters",
            metersX);
        double metersY = Math.tan(realYaw.getRadians()) * metersX;
        Logger.getInstance().recordOutput("Cube/CubeDistanceXMeters", metersY);
        frc.robot.RobotState.getInstance().setCubePose(realYaw, metersX, metersY);
        //   Logger.getInstance().recordOutput("Cube/Y", cubeCenter.y);
        //   Logger.getInstance().recordOutput("Cube/X", cubeCenter.x);
        // }
      }
    }

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
    if (distance > 4) {
      return VecBuilder.fill(100, 100, 100);
    }
    return VecBuilder.fill(distance * 0.15, distance * 0.15, 50);
  }

  public Vector<N3> getMatrixStdsOneTag(EstimatedRobotPose curRobotPose, PhotonPipelineResult result) {
    if (RobotState.isDisabled()) {
      return VecBuilder.fill(3, 3, 10);
    }
    Optional<Pose3d> tagPose = m_layout.getTagPose(result.getBestTarget().getFiducialId());
    if (tagPose.isEmpty()) {
      return VecBuilder.fill(100, 100, 100);
    }
    Pose2d finalTagPose = tagPose.get().toPose2d();
    double distance = finalTagPose.getTranslation().getDistance(curRobotPose.estimatedPose.toPose2d().getTranslation());
    if (distance > 4) {
      return VecBuilder.fill(100, 100, 100);
    }
    return VecBuilder.fill(distance * 0.35, distance * 0.35, 85);
  }

  public PhotonPipelineResult getPipelineResult() {
    return m_result;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(getPipelineResult().getBestTarget());
  }

  public void setPipeline(int pipelineNumber) {
    m_photonCamera.setPipelineIndex(pipelineNumber);
  }

  public Command switchToCubeSearchPipelineCommand() {
    return runEnd(() -> {
      setPipeline(VisionConstants.kCubeSearchPipelineIndex);
    }, () -> {
      setPipeline(VisionConstants.kAprilTagPipelineIndex);
    });

  }

}

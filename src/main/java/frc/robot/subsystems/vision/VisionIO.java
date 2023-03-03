package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;
import frc.robot.subsystems.vision.VisionIOLimelight.LimelightPipeline;

public interface VisionIO extends LoggedIO<VisionInputs> {
  @AutoLog
  public static class VisionInputs {
    public String cameraName;
    public boolean hasTarget;
    public boolean isTargetAprilTag;
    public double targetX;
    public double targetY;
    public double targetArea;
    public double targetSkew;
    public double targetLatency;
    public double FId;
  }

  // initialize the camera
  public void initPhotonPoseEstimator(Transform3d cameraToRobot, AprilTagFieldLayout tagLayout,
      PoseStrategy poseStrategy);

  // LimeLight only
  public void setLEDMode(VisionLEDMode mode);

  public void setPipeline(LimelightPipeline pipeline);

  public PhotonPipelineResult getPipelineResult();

  public Optional<EstimatedRobotPose> getEstimatedRobotPose();

}

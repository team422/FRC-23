package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraLimelight {
  PhotonCamera m_cam;
  Transform3d m_transform;

  public CameraLimelight(String cameraName, Transform3d transformToRobot) {
    m_cam = new PhotonCamera(cameraName);
    m_transform = transformToRobot;
  }

  public void startLights() {
    m_cam.setLED(VisionLEDMode.kOn);
  }

  public void stopLights() {
    m_cam.setLED(VisionLEDMode.kOff);
  }

  public Rotation2d findPoleYaw() {
    PhotonPipelineResult res = m_cam.getLatestResult();
    if (res.hasTargets()) {
      PhotonTrackedTarget bestTarget = res.getBestTarget();
      return Rotation2d.fromDegrees(bestTarget.getYaw());
    } else {
      return null;
    }
  }

}

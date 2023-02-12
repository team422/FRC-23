package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PhotonSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;

  public PhotonSubsystem(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    m_robotToCamera = robotToCamera;
  }

  public PhotonCamera getCamera() {
    return m_camera;
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
}

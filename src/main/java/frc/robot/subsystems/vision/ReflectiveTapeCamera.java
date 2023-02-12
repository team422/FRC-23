package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class ReflectiveTapeCamera extends PhotonSubsystem {

  public ReflectiveTapeCamera(String cameraName, Transform3d robotToCamera) {
    super(cameraName, robotToCamera);
  }
}

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
  public VisionIO m_io;

  public Vision(VisionIO io, Transform3d cameraToRobot) {
    m_io = io;

  }

}

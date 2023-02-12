package frc.lib.utils;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class GeometryUtil {
  public static Pose2d averagePoses(Pose2d... poses) {
    if (poses.length == 1) {
      return poses[0];
    }

    double resultX = 0;
    double resultY = 0;
    double resultTheta = 0;

    for (Pose2d pose : poses) {
      resultX += pose.getX();
      resultY += pose.getY();
      resultTheta += pose.getRotation().getRadians();
    }

    resultX /= poses.length;
    resultY /= poses.length;
    resultTheta /= poses.length;

    return new Pose2d(resultX, resultY, new Rotation2d(resultTheta));
  }

  public static Pose3d from2dTo3d(Pose2d pose) {
    return new Pose3d(pose.getX(), pose.getY(), 0, from2dTo3d(pose.getRotation()));
  }

  public static Transform3d from2dTo3d(Transform2d transform) {
    return new Transform3d(from2dTo3d(transform.getTranslation()), from2dTo3d(transform.getRotation()));
  }

  public static Translation3d from2dTo3d(Translation2d translation) {
    return new Translation3d(translation.getX(), translation.getY(), 0);
  }

  public static Rotation3d from2dTo3d(Rotation2d angle) {
    return new Rotation3d(0, 0, angle.getRadians());
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   * 
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static Transform3d poseToTransform(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   * 
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
  * Creates a pure translating transform
  * 
  * @param translation The translation to create the transform with
  * @return The resulting transform
  */
  public static Transform2d transformFromTranslation(
      Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   * 
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d transformFromTranslation(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Performs the intrinsic rotation of yaw followed by pitch
   * @param yaw
   * @param pitch
   * @return
   */
  public static Rotation3d yawPitchRotation(double yaw, double pitch) {
    var yawRot = new Rotation3d(0, 0, yaw);
    var pitchRot = new Rotation3d(0, pitch, 0);

    return yawRot.rotateBy(pitchRot);
  }

  public static Map<Integer, Pose3d> aprilTagPoseMap(AprilTagFieldLayout layout) {
    return layout.getTags().stream()
        .collect(Collectors.toMap(
            x -> x.ID,
            x -> layout.getTagPose(x.ID).orElseThrow()));
  }
}

package frc.lib.utils;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class GeometryUtil {
  public static final Vector<N3> kUp = VecBuilder.fill(0, 0, 1);
  public static final Vector<N3> kLeft = VecBuilder.fill(0, 1, 0);
  public static final Vector<N3> kForward = VecBuilder.fill(1, 0, 0);

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
  public static Transform2d translationToTransform(
      Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  public static Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.fromDegrees(0));
  }

  public static Pose2d translationToPose(double x, double y) {
    return translationToPose(new Translation2d(x, y));
  }

  /**
   * Creates a pure translating transform
   * 
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
  * Creates a pure translating transform
  * 
  * @param translation The translation to create the transform with
  * @return The resulting transform
  */
  public static Transform3d translationToTransform(
      Translation3d translation) {
    return new Transform3d(translation, new Rotation3d());
  }

  /**
   * Creates a pure translating transform
   * 
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform3d translationToTransform(double x, double y, double z) {
    return new Transform3d(new Translation3d(x, y, z), new Rotation3d());
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

  public static Vector<N3> cross(Matrix<N3, N1> v1, Matrix<N3, N1> v2) {
    return VecBuilder.fill(
        v1.get(1, 0) * v2.get(2, 0) - v2.get(1, 0) * v1.get(2, 0),
        v2.get(0, 0) * v1.get(2, 0) - v1.get(0, 0) * v2.get(2, 0),
        v1.get(0, 0) * v2.get(1, 0) - v2.get(0, 0) * v1.get(1, 0));
  }

  public static Vector<N3> vector(Quaternion q) {
    return VecBuilder.fill(q.getX(), q.getY(), q.getZ());
  }

  public static Vector<N3> rotateVector(Matrix<N3, N1> v, Quaternion q) {
    // https://stackoverflow.com/a/58077034
    // v' = v + 2 * r x (s * v + r x v) / m
    double s = q.getW();
    double x = q.getX();
    double y = q.getY();
    double z = q.getZ();
    Vector<N3> r = vector(q);
    var sv = v.times(s);
    Vector<N3> rxv = cross(r, v);
    Vector<N3> rxsvrxv = cross(r, sv.plus(rxv));
    double m = s * s + x * x + y * y + z * z;
    var result = v.plus(rxsvrxv.times(2 / m));
    return new Vector<>(result);
  }
}

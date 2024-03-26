// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.NorthStarVision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.utils.GeomUtil;
import frc.lib.utils.PoseEstimator.TimestampedVisionUpdate;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.NorthStarVision.AprilTagVisionIO.AprilTagVisionIOInputs;

public class AprilTagVision extends VirtualSubsystem {
  private static final double ambiguityThreshold = 0.15;
  private static final double targetLogTimeSecs = 0.1;
  private static final double fieldBorderMargin = 0.5;
  private static final Pose3d[] cameraPoses;
  private static final double xyStdDevCoefficient;
  private static final double thetaStdDevCoefficient;

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;
  private HashMap<Integer, ArrayList<Pose3d>> fieldMap = new HashMap<>();

  class Pose3dJson {
    public float timestamp;
    public double x;
    public double y;
    public double z;
    public double qx;
    public double qy;
    public double qz;
    public double qw;
  }

  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {
  };
  private Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  static {
    if (Robot.isReal()) {
      cameraPoses = new Pose3d[] {
          // left
          new Pose3d(Constants.VisionConstants.kRightCameraTransform.getTranslation(),
              Constants.VisionConstants.kRightCameraTransform.getRotation()),
          new Pose3d(Constants.VisionConstants.kleftCameraTransform.getTranslation(),
              Constants.VisionConstants.kleftCameraTransform.getRotation()),

          // right

      };
      xyStdDevCoefficient = 0.01;
      thetaStdDevCoefficient = 0.01;

    } else {
      cameraPoses = new Pose3d[] {
          new Pose3d(Constants.VisionConstants.kRightCameraTransform.getTranslation(),
              Constants.VisionConstants.kRightCameraTransform.getRotation()),
          new Pose3d(Constants.VisionConstants.kleftCameraTransform.getTranslation(),
              Constants.VisionConstants.kleftCameraTransform.getRotation()),
      };
      xyStdDevCoefficient = 0.01;
      thetaStdDevCoefficient = 0.01;

    }
  }

  public AprilTagVision(AprilTagVisionIO... io) {
    this.io = io;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    // Create map of last detection times for tags

    FieldConstants.getAprilTags()
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
  }

  public void setDataInterface(Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.visionConsumer = visionConsumer;
  }

  public void periodic() {

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
    }
    boolean creatingFieldMap = false;
    // System.out.println("TESTESTSETS");
    for (int i = 0; i < io.length; i++) {
      if (io[i].isCreatingFieldMap()) {
        creatingFieldMap = true;
      }
    }

    if (creatingFieldMap) {
      // Create field map

      // Loop over instances
      for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {

        // check if 
        //     //     final_json["translation"] = {}
        // final_json["translation"]["x"] = pose.translation().X()
        // final_json["translation"]["y"] = pose.translation().Y()
        // final_json["translation"]["z"] = pose.translation().Z()
        // final_json["rotation"] = {}
        // final_json["rotation"]["quaternion"] = {}
        // final_json["rotation"]["quaternion"]["W"] = pose.rotation().getQuaternion().W()
        // final_json["rotation"]["quaternion"]["X"] = pose.rotation().getQuaternion().X()
        // final_json["rotation"]["quaternion"]["Y"] = pose.rotation().getQuaternion().Y()
        // final_json["rotation"]["quaternion"]["Z"] = pose.rotation().getQuaternion().Z()
        // convert from json to pose3d 
        // add to field map
        String json = io[instanceIndex].getCreatingFieldJson();
        // System.out.println(json);
        // convert jsons to pose3d
        if (json != null) {
          JsonObject poses = JsonParser.parseString(json).getAsJsonObject();

          for (Map.Entry<String, JsonElement> entry : poses.entrySet()) {
            String key = entry.getKey();
            // System.out.println(key);

            if (key.charAt(0) != 't') {
              JsonObject value = entry.getValue().getAsJsonObject();
              Pose3d poseRelativeToCamera = new Pose3d(
                  new Translation3d(value.get("x").getAsDouble(), value.get("y").getAsDouble(),
                      value.get("z").getAsDouble()),

                  new Rotation3d(new Quaternion(value.get("qx").getAsDouble(), value.get("qy").getAsDouble(),
                      value.get("qy").getAsDouble(), value.get("qz").getAsDouble())));

              // Pose3d tagPose = new Pose3d(RobotState.getInstance().getPose())
              //     .relativeTo(cameraPoses[instanceIndex].transformBy(poseRelativeToCamera));
              // System.out.println(pose);
              Pose3d tagPose = poseRelativeToCamera
                  .transformBy(new Transform3d(cameraPoses[instanceIndex], new Pose3d()))
                  .plus(new Transform3d(new Pose3d(RobotState.getInstance().getPose()), new Pose3d()));
              // Pose3d actPose = cameraPoses[instanceIndex].relativeTo(pose);

              // Pose3d tagPose = new Pose3d(RobotState.getInstance().getPose()).plus(pose);

              // Logger.getInstance().recordOutput("plsworkworkwork", pose);
              Integer arrayKey = Integer.parseInt(key);
              if (fieldMap.get(arrayKey) == null) {
                fieldMap.put(arrayKey, new ArrayList<Pose3d>());
              }
              fieldMap.get(arrayKey).add(tagPose);
              // System.out.println(fieldMap);
              Logger.getInstance().recordOutput("tagPose" + arrayKey.toString(), tagPose);
            }
          }
        }

      }

      return;
    }

    // Loop over instances

    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {

      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        var values = inputs[instanceIndex].frames[frameIndex];

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose = new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose = cameraPose
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();
            Logger.getInstance().recordOutput("AprilTagROBOTPose", robotPose);
            Logger.getInstance().recordOutput("AprilTagCAMERAPose", cameraPose);
            break;

          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 = new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 = new Pose3d(
                values[10],
                values[11],
                values[12],
                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose2d robotPose0 = cameraPose0
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();
            Pose2d robotPose1 = cameraPose1
                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                .toPose2d();

            // Select pose using projection errors
            if (error0 < error1 * ambiguityThreshold) {
              cameraPose = cameraPose0;
              robotPose = robotPose0;
            } else if (error1 < error0 * ambiguityThreshold) {
              cameraPose = cameraPose1;
              robotPose = robotPose1;
            }
            // System.out.println()
            if (cameraPose != null && robotPose != null) {
              Logger.getInstance().recordOutput("AprilTagROBOTPose", robotPose);
              Logger.getInstance().recordOutput("AprilTagCAMERAPose", cameraPose);
            }

            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose.getX() < -fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose.getY() < -fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
          continue;
        }

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = FieldConstants.getAprilTags().getTagPose((int) values[i]);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add to vision updates
        double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPoses.size();
        visionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);

        // Log data from instance
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/LatencySecs",
                Timer.getFPGATimestamp() - timestamp);
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose", robotPose);
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/RobotPose",
                new double[] {});
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        Logger.getInstance()
            .recordOutput(
                "AprilTagVision/Inst" + Integer.toString(instanceIndex) + "/TagPoses",
                new double[] {});
      }
    }

    // Log robot poses
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        allTagPoses.add(FieldConstants.getAprilTags().getTagPose(detectionEntry.getKey()).get());
      }
    }
    Logger.getInstance()
        .recordOutput(
            "AprilTagVision/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));

    // Send results to pose esimator
    visionConsumer.accept(visionUpdates);
  }

  public AprilTagFieldLayout createAprilTagFieldLayoutFromFieldMap() {
    List<AprilTag> curLayout = new ArrayList<>();
    for (Map.Entry<Integer, ArrayList<Pose3d>> entry : fieldMap.entrySet()) {
      curLayout.add(new AprilTag(entry.getKey(), GeomUtil.average3dPoses(entry.getValue())));
    }
    AprilTagFieldLayout m_layout = new AprilTagFieldLayout(curLayout, FieldConstants.fieldLength,
        FieldConstants.fieldWidth);
    fieldMap = new HashMap<Integer, ArrayList<Pose3d>>();
    for (AprilTag tag : m_layout.getTags()) {
      Logger.getInstance().recordOutput("AprilTagFieldLayout/tagAveraged" + tag.ID, tag.pose);
    }
    return m_layout;
  }

}

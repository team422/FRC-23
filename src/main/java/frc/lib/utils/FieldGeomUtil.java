package frc.lib.utils;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class FieldGeomUtil {
  HashMap<String, ExtendedPathPoint> allPoints = new HashMap<>();
  public HashMap<String, Pose3d> allNodes = new HashMap<>();
  Alliance m_allianceColor;

  public FieldGeomUtil() {
    m_allianceColor = DriverStation.getAlliance();

    // allPoints.put("blueLeftWallLoadingStation", Constants.Setpoints.blueLeftWallLoadingStation);
    // allPoints.put("blueRightWallLoadingStation", Constants.Setpoints.blueRightWallLoadingStation);
    allPoints.put("blueFirstGridLeftCone", Constants.Setpoints.blueFirstGridLeftCone);
    allPoints.put("blueFirstGridCube", Constants.Setpoints.blueFirstGridCube);
    allPoints.put("blueFirstGridRightCone", Constants.Setpoints.blueFirstGridRightCone);
    allPoints.put("blueSecondGridLeftCone", Constants.Setpoints.blueSecondGridLeftCone);
    allPoints.put("blueSecondGridCube", Constants.Setpoints.blueSecondGridCube);
    allPoints.put("blueSecondGridRightCone", Constants.Setpoints.blueSecondGridRightCone);
    allPoints.put("blueThirdGridLeftCone", Constants.Setpoints.blueThirdGridLeftCone);
    allPoints.put("blueThirdGridCube", Constants.Setpoints.blueThirdGridCube);
    allPoints.put("blueThirdGridRightCone", Constants.Setpoints.blueThirdGridRightCone);
    allPoints.put("blueLeftOfBalance", Constants.Setpoints.blueLeftOfBalance);
    allPoints.put("blueRightOfBalance", Constants.Setpoints.blueRightOfBalance);
    allPoints.put("bluePreLoadingStation", Constants.Setpoints.bluePreLoadingStation);

    allNodes.put("blueFirstGridLeftMid", new Pose3d(0.86, 0.50, 1, new Rotation3d()));
    allNodes.put("blueFirstGridLeftHigh", new Pose3d(0.45, 0.50, 1.3, new Rotation3d()));
    allNodes.put("blueFirstGridCubeHigh", new Pose3d(0.45, 1.08, 1.5, new Rotation3d()));
    allNodes.put("blueFirstGridCubeMid", new Pose3d(0.47, 1.08, .98, new Rotation3d()));
    allNodes.put("blueFirstGridRightMid", new Pose3d(.86, 1.64, 1, new Rotation3d()));
    allNodes.put("blueFirstGridRightHigh", new Pose3d(0.45, 1.64, 1.3, new Rotation3d()));

    allNodes.put("blueSecondGridLeftMid", new Pose3d(0.86, 2.20, 1, new Rotation3d()));
    allNodes.put("blueSecondGridLeftHigh", new Pose3d(0.45, 2.20, 1.3, new Rotation3d()));
    allNodes.put("blueSecondGridCubeHigh", new Pose3d(0.45, 2.78, 1.5, new Rotation3d()));
    allNodes.put("blueSecondGridCubeMid", new Pose3d(0.47, 2.78, .98, new Rotation3d()));
    allNodes.put("blueSecondGridRightMid", new Pose3d(.86, 3.34, 1, new Rotation3d()));
    allNodes.put("blueSecondGridRightHigh", new Pose3d(0.45, 3.34, 1.3, new Rotation3d()));

    allNodes.put("blueThirdGridLeftMid", new Pose3d(0.86, 3.90, 1, new Rotation3d()));
    allNodes.put("blueThirdGridLeftHigh", new Pose3d(0.45, 3.90, 1.3, new Rotation3d()));
    allNodes.put("blueThirdGridCubeHigh", new Pose3d(0.45, 4.48, 1.5, new Rotation3d()));
    allNodes.put("blueThirdGridCubeMid", new Pose3d(0.47, 4.48, .98, new Rotation3d()));
    allNodes.put("blueThirdGridRightMid", new Pose3d(.86, 5.04, 1, new Rotation3d()));
    allNodes.put("blueThirdGridRightHigh", new Pose3d(0.45, 5.04, 1.3, new Rotation3d()));

    Logger.getInstance().recordOutput("blueFirstGridLeftMid", allNodes.get("blueFirstGridLeftMid"));
    Logger.getInstance().recordOutput("blueFirstGridLeftHigh", allNodes.get("blueFirstGridLeftHigh"));
    Logger.getInstance().recordOutput("blueSecondGridLeftMid", allNodes.get("blueSecondGridLeftMid"));
    Logger.getInstance().recordOutput("blueSecondGridLeftHigh", allNodes.get("blueSecondGridLeftHigh"));
    Logger.getInstance().recordOutput("blueSecondCubeHigh", allNodes.get("blueSecondGridCubeHigh"));
    Logger.getInstance().recordOutput("blueSecondCubeMid", allNodes.get("blueSecondGridCubeMid"));
    Logger.getInstance().recordOutput("blueSecondGridRightMid", allNodes.get("blueSecondGridRightMid"));
    Logger.getInstance().recordOutput("blueSecondGridRightHigh", allNodes.get("blueSecondGridRightHigh"));
    Logger.getInstance().recordOutput("blueThirdGridLeftMid", allNodes.get("blueThirdGridLeftMid"));
    Logger.getInstance().recordOutput("blueThirdGridLeftHigh", allNodes.get("blueThirdGridLeftHigh"));
    Logger.getInstance().recordOutput("blueThirdCubeHigh", allNodes.get("blueThirdGridCubeHigh"));
    Logger.getInstance().recordOutput("blueThirdCubeMid", allNodes.get("blueThirdGridCubeMid"));
    Logger.getInstance().recordOutput("blueThirdGridRightMid", allNodes.get("blueThirdGridRightMid"));
    Logger.getInstance().recordOutput("blueThirdGridRightHigh", allNodes.get("blueThirdGridRightHigh"));

    // HashMap<String, ExtendedPathPoint> redSide = new HashMap<String, ExtendedPathPoint>();
    // redSide.put("redLeftWallLoadingStation", Constants.SetpointConstants.redLeftWallLoadingStation);
    // redSide.put("redRightWallLoadingStation", Constants.SetpointConstants.redRightWallLoadingStation);
    // redSide.put("redFirstGridLeftCone", Constants.SetpointConstants.redFirstGridLeftCone);
    // redSide.put("redFirstGridCube", Constants.SetpointConstants.redFirstGridCube);
    // redSide.put("redFirstGridRightCone", Constants.SetpointConstants.redFirstGridRightCone);
    // redSide.put("redSecondGridLeftCone", Constants.SetpointConstants.redSecondGridLeftCone);
    // redSide.put("redSecondGridCube", Constants.SetpointConstants.redSecondGridCube);
    // redSide.put("redSecondGridRightCone", Constants.SetpointConstants.redSecondGridRightCone);
    // redSide.put("redThirdGridLeftCone", Constants.SetpointConstants.redThirdGridLeftCone);
    // redSide.put("redThirdGridCube", Constants.SetpointConstants.redThirdGridCube);
    // redSide.put("redThirdGridRightCone", Constants.SetpointConstants.redThirdGridRightCone);
    // redSide.put("redLeftOfBalance", Constants.SetpointConstants.redLeftOfBalance);
    // redSide.put("redRightOfBalance", Constants.SetpointConstants.redRightOfBalance);
    // redSide.put("redPreLoadingStation", Constants.SetpointConstants.redPreLoadingStation);

    // allPoints.put("blue", blueSide);
    // allPoints.put("red", redSide);

  }

  public Pose3d getScoringPose(String scoringPoseName) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return flipSidePose3d(allNodes.get(scoringPoseName));
    }
    return allNodes.get(scoringPoseName);
  }

  public Pose3d getClosestScoringPose(Pose2d curPose, int m_height) {
    Pose3d closestPose = null;
    Pose2d projectedPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    if (m_height == 1) {
      projectedPose = new Pose2d(1,
          curPose.getY() + (curPose.getX() - 1) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (m_height == 2) {
      projectedPose = new Pose2d(0.86,
          curPose.getY() + (curPose.getX() - .86) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (m_height == 3) {
      projectedPose = new Pose2d(0.46,
          curPose.getY() + (curPose.getX() - .46) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      projectedPose = flipSidePose2d(projectedPose);
    }

    Logger.getInstance().recordOutput("projected pose", projectedPose);

    for (String poseName : allNodes.keySet()) {
      Pose2d desPose = allNodes.get(poseName).toPose2d();
      if (poseName.contains("High") && m_height != 3) {
        continue;
      }
      if (poseName.contains("Mid") && m_height != 2) {
        continue;
      }
      if (poseName.contains("Low") && m_height != 1) {
        continue;
      }
      // project the current pose onto the y axis of the desired pose based on the angle of the desired pose

      double distance = Math.abs(projectedPose.getY() - desPose.getY());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = allNodes.get(poseName);
        frc.robot.RobotState.getInstance().setClosestScoringPoseName(poseName);
      }
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      return flipSidePose3d(closestPose);
    }
    return closestPose;
  }

  public boolean overConesOrCubes(Pose3d pose) {
    // return false;
    if ((pose.getX() < 1 || pose.getX() > 15.5) && pose.getY() < 6) {
      return true;
    }
    return false;
  }

  public ArrayList<ExtendedPathPoint> fastestPathToLoadingStation(Pose2d curPose, Alliance allianceColor,
      int loadingStationNumber) {
    // do all math on blue side and flip if necessary
    ArrayList<ExtendedPathPoint> fastestPath = new ArrayList<>();
    if (DriverStation.getAlliance() == Alliance.Red) {
      curPose = flipSidePose2d(curPose);
    }

    if (loadingStationNumber == 1) {
      fastestPath.add(0, allPoints.get("blueLeftWallLoadingStation"));
    } else if (loadingStationNumber == 2) {
      fastestPath.add(0, allPoints.get("blueRightWallLoadingStation"));
    }

    if (curPose.getX() < 12 && curPose.getY() < 6) {
      fastestPath.add(0, allPoints.get("bluePreLoadingStation"));

    }

    if (curPose.getX() < 2.5 && curPose.getY() < 2) {
      fastestPath.add(0, allPoints.get("blueRightOfBalance"));
    }
    if (curPose.getX() < 2.5 && curPose.getY() >= 2) {
      fastestPath.add(0, allPoints.get("blueLeftOfBalance"));
    }

    if (DriverStation.getAlliance() == Alliance.Red) {
      int i = 0;
      for (ExtendedPathPoint point : fastestPath) {
        fastestPath.set(i, point.flipPathPoint());
        i += 1;
      }
    }
    return fastestPath;
  }

  public ExtendedPathPoint getClosestNode(Pose2d curPose) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      curPose = flipSidePose2d(curPose);
    }
    double lowestDistance = 10;
    ExtendedPathPoint desPoint = new ExtendedPathPoint(curPose.getTranslation(), curPose.getRotation(),
        curPose.getRotation());
    for (ExtendedPathPoint nodePoint : allPoints.values()) {
      double distance = curPose.getTranslation().getDistance(nodePoint.getTranslation());
      if (distance < lowestDistance) {
        lowestDistance = distance;
        desPoint = nodePoint;
      }
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      return desPoint.flipPathPoint();
    }
    return desPoint;

  }

  public ArrayList<ExtendedPathPoint> fastestPathToGamePieceDropoff(Pose2d curPose, int gridNumber, int pieceNum,
      Alliance allianceColor) {
    ArrayList<ExtendedPathPoint> fastestPath = new ArrayList<>();
    if (DriverStation.getAlliance() == Alliance.Red) {
      curPose = flipSidePose2d(curPose);
    }

    if (DriverStation.getAlliance() == Alliance.Red) {
      int i = 0;
      for (ExtendedPathPoint point : fastestPath) {
        fastestPath.set(i, point.flipPathPoint());
        i += 1;
      }
    }
    return fastestPath;

  }

  public Pose2d flipSidePose2d(Pose2d startPose) {

    return new Pose2d(startPose.getX(),
        FieldConstants.kFieldWidthMeters - startPose.getY(),
        startPose.getRotation());
  }

  public Pose3d flipSidePose3d(Pose3d startPose) {
    return new Pose3d(startPose.getX(),
        FieldConstants.kFieldWidthMeters - startPose.getY(), startPose.getZ(),
        startPose.getRotation());
  }

}

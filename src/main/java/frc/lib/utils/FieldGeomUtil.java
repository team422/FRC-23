package frc.lib.utils;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.Constants;

public class FieldGeomUtil {
  HashMap<String, ExtendedPathPoint> allPoints = new HashMap<>();

  public FieldGeomUtil() {

    allPoints.put("blueLeftWallLoadingStation", Constants.SetpointConstants.blueLeftWallLoadingStation);
    allPoints.put("blueRightWallLoadingStation", Constants.SetpointConstants.blueRightWallLoadingStation);
    allPoints.put("blueFirstGridLeftCone", Constants.SetpointConstants.blueFirstGridLeftCone);
    allPoints.put("blueFirstGridCube", Constants.SetpointConstants.blueFirstGridCube);
    allPoints.put("blueFirstGridRightCone", Constants.SetpointConstants.blueFirstGridRightCone);
    allPoints.put("blueSecondGridLeftCone", Constants.SetpointConstants.blueSecondGridLeftCone);
    allPoints.put("blueSecondGridCube", Constants.SetpointConstants.blueSecondGridCube);
    allPoints.put("blueSecondGridRightCone", Constants.SetpointConstants.blueSecondGridRightCone);
    allPoints.put("blueThirdGridLeftCone", Constants.SetpointConstants.blueThirdGridLeftCone);
    allPoints.put("blueThirdGridCube", Constants.SetpointConstants.blueThirdGridCube);
    allPoints.put("blueThirdGridRightCone", Constants.SetpointConstants.blueThirdGridRightCone);
    allPoints.put("blueLeftOfBalance", Constants.SetpointConstants.blueLeftOfBalance);
    allPoints.put("blueRightOfBalance", Constants.SetpointConstants.blueRightOfBalance);
    allPoints.put("bluePreLoadingStation", Constants.SetpointConstants.bluePreLoadingStation);
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

  public boolean overConesOrCubes(Pose3d pose) {
    return false;
    // if ((pose.getX() < 1 || pose.getX() > 15.5) && pose.getY() < 6) {
    //   return true;
    // }
    // return false;
  }

  public ArrayList<ExtendedPathPoint> fastestPathToLoadingStation(Pose2d curPose, Alliance allianceColor,
      int loadingStationNumber) {
    // do all math on blue side and flip if necessary
    ArrayList<ExtendedPathPoint> fastestPath = new ArrayList<>();
    if (allianceColor == Alliance.Red) {
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

    if (allianceColor == Alliance.Red) {
      int i = 0;
      for (ExtendedPathPoint point : fastestPath) {
        fastestPath.set(i, point.flipPathPoint());
        i += 1;
      }
    }
    return fastestPath;
  }

  public ArrayList<ExtendedPathPoint> fastestPathToGamePieceDropoff(Pose2d curPose, int gridNumber, int pieceNum,
      Alliance allianceColor) {
    ArrayList<ExtendedPathPoint> fastestPath = new ArrayList<>();
    if (allianceColor == Alliance.Red) {
      curPose = flipSidePose2d(curPose);
    }

    if (allianceColor == Alliance.Red) {
      int i = 0;
      for (ExtendedPathPoint point : fastestPath) {
        fastestPath.set(i, point.flipPathPoint());
        i += 1;
      }
    }
    return fastestPath;

  }

  public Pose2d flipSidePose2d(Pose2d startPose) {
    Rotation2d newRotation = Rotation2d.fromDegrees(180).minus(startPose.getRotation());
    if (newRotation.getDegrees() < 0) {
      newRotation = newRotation.plus(Rotation2d.fromDegrees(360));
    }

    return new Pose2d(Constants.FieldConstants.kFieldLengthMeters - startPose.getX(), startPose.getY(),
        newRotation);
  }

}

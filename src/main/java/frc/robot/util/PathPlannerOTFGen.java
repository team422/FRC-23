package frc.robot.util;

import java.util.HashMap;

import com.pathplanner.lib.PathPoint;

public class PathPlannerOTFGen {
  // make hashmap of important points
  HashMap<String, PathPoint[]> important_points = new HashMap<String, PathPoint[]>();

  public PathPlannerOTFGen() {

  }

  // public PathPlannerTrajectory generatePath(PathPoint[] start, PathPoint[] end, PathConstraints constraints) {
  //   // generate path
  //   List<PathPoint> pathPoints = { start, end };
  //   PathPlannerTrajectory path = PathPlanner.generatePath(constraints, pathPoints);
  //   return path;
  // }

  public void addImportantPoint(String name, PathPoint[] points) {
    important_points.put(name, points);
  }
  // PathPoint

}

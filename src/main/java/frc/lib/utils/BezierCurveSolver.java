package frc.lib.utils;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class BezierCurveSolver {
  PathPoint[] m_points;
  ChassisSpeeds[] m_speeds;

  public BezierCurveSolver(PathPoint[] points, ChassisSpeeds[] speeds) {
    m_points = points;
    m_speeds = speeds;
  }

  /** t should be from 0 to 1 **/
  public ChassisSpeeds calculateVelocityAndAccelAtPercentage(double t) {
    return new ChassisSpeeds();
  }

  public static double[] solveQuadratic(double a, double b, double c) {
    double[] solutions = new double[2];
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
      return null;
    }
    solutions[0] = (-b + Math.sqrt(discriminant)) / (2 * a);
    solutions[1] = (-b - Math.sqrt(discriminant)) / (2 * a);
    return solutions;
  }
}

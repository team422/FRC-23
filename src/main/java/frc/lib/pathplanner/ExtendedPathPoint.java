package frc.lib.pathplanner;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class ExtendedPathPoint extends PathPoint {
  public Translation2d m_pose;
  public Rotation2d m_heading;
  public Rotation2d m_holonomicHeading;

  public ExtendedPathPoint(Translation2d pose, Rotation2d heading, Rotation2d holRotation2d) {
    super(pose, heading, heading);
    m_pose = pose;
    m_heading = heading;
    m_holonomicHeading = holRotation2d;
  }

  public Translation2d getTranslation() {
    return m_pose;
  }

  public Rotation2d getRotation() {
    return m_heading;
  }

  public Rotation2d getHolonomicRotation2d() {
    return m_holonomicHeading;
  }

  public ExtendedPathPoint flipPathPoint() {
    // flips current path point about the X axis
    Rotation2d newHeading = Rotation2d.fromDegrees(180).minus(m_heading);
    if (newHeading.getDegrees() < 0) {
      newHeading = newHeading.plus(Rotation2d.fromDegrees(360));
    }
    Rotation2d newHolonomicHeading = Rotation2d.fromDegrees(180).minus(m_holonomicHeading);
    if (newHolonomicHeading.getDegrees() < 0) {
      newHolonomicHeading = newHolonomicHeading.plus(Rotation2d.fromDegrees(360));
    }
    return new ExtendedPathPoint(new Translation2d(FieldConstants.kFieldLengthMeters - m_pose.getX(), m_pose.getY()),
        newHeading, newHolonomicHeading);
  }

}

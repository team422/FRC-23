package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class RobotState {
  public Drive m_drive;
  public Intake m_intake;
  public Elevator m_elevator;
  public Wrist m_wrist;

  public double elevatorYMeters;
  public double elevatorXMeters;
  public Rotation2d wristAngleRotation2d;
  public Rotation2d realWantedWristRotation2d;

  public RobotState(Drive drive, Intake intake, Elevator elevator, Wrist wrist) {
    m_drive = drive;
    m_intake = intake;
    m_elevator = elevator;
    m_wrist = wrist;

  }

  public void update() {
    elevatorYMeters = m_elevator.getPositionYMeters();
    elevatorXMeters = m_elevator.getPositionXMeters();
    wristAngleRotation2d = m_wrist.getAngle();
    realWantedWristRotation2d = m_wrist.userWantedAngle;
    checkIfWristBreak();
    Pose3d armPosition = getArmPosition(m_drive.getPose(), m_elevator.getPositionXMeters(),
        m_elevator.getPositionYMeters(), wristAngleRotation2d,
        new Transform3d(new Translation3d(ElevatorConstants.armLength, 0, 0),
            new Rotation3d(0, wristAngleRotation2d.getRadians(), 0)),
        new Translation3d(Units.inchesToMeters(7), 0, 0));
    Logger.getInstance().recordOutput("RobotState/ArmSpot", armPosition);
  }

  public Pose3d getArmPosition(Pose2d robotPose, double elevatorXMeters, double elevatorYMeters,
      Rotation2d wristAngleRotation2d, Transform3d armLength, Translation3d intake) {
    Pose3d pose3d = new Pose3d(robotPose);
    pose3d = pose3d.plus(new Transform3d(new Translation3d(elevatorXMeters, 0, elevatorYMeters), new Rotation3d()));
    pose3d = pose3d.plus(armLength);
    pose3d = pose3d.plus(new Transform3d(intake, new Rotation3d()));
    pose3d.transformBy(new Transform3d(new Translation3d(), new Rotation3d(wristAngleRotation2d.getRadians(), 0, 0)));

    return pose3d;

  }

  public void checkIfWristBreak() {
    if (Units.metersToInches(elevatorYMeters) < 8 && wristAngleRotation2d.getDegrees() < 0) {
      m_wrist.setAngleToNotBreak(Rotation2d.fromDegrees(0));
    } else {
      m_wrist.setAngleToNotBreak(realWantedWristRotation2d);

    }
  }

}

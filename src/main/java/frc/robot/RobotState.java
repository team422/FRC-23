package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  }

  public void checkIfWristBreak() {
    if (Units.metersToInches(elevatorYMeters) < 8 && wristAngleRotation2d.getDegrees() < 0) {
      m_wrist.setAngleToNotBreak(Rotation2d.fromDegrees(0));
    } else {
      m_wrist.setAngleToNotBreak(realWantedWristRotation2d);

    }
  }

}

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.lib.utils.FieldGeomUtil;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.setpoint.Setpoint;

public class RobotState {

  private static RobotState instance;
  public Drive m_drive;
  public Intake m_intake;
  public Elevator m_elevator;
  public Wrist m_wrist;

  public Pose3d m_armPosition;
  public Pose3d m_wristPosition;
  public Pose3d m_intakePosition;
  public Pose3d m_elevatorPosition;

  public double elevatorYMeters;
  public double elevatorXMeters;
  public Rotation2d wristAngleRotation2d;
  public Rotation2d realWantedWristRotation2d;
  public FieldGeomUtil fieldGeomUtil = new FieldGeomUtil();
  public int m_poseSetpoint;

  private RobotState(Drive drive, Intake intake, Elevator elevator, Wrist wrist) {
    m_drive = drive;
    m_intake = intake;
    m_elevator = elevator;
    m_wrist = wrist;
    m_poseSetpoint = 5;
  }

  public static RobotState startInstance(Drive drive, Intake intake, Elevator elevator, Wrist wrist) {
    if (instance == null) {
      instance = new RobotState(drive, intake, elevator, wrist);
    }
    return instance;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      throw new RuntimeException("RobotState not initialized");
    }
    return instance;
  }

  public void increasePoseSetpoint() {
    if (m_poseSetpoint < 9) {
      m_poseSetpoint += 1;
    }
  }

  public void decreasePoseSetpoint() {
    if (m_poseSetpoint > 1) {
      m_poseSetpoint -= 1;
    }
  }

  public ExtendedPathPoint getPoseSetpoint() {
    Alliance alliance = DriverStation.getAlliance();
    ExtendedPathPoint point = Setpoints.blueFirstGridCube;
    if (m_poseSetpoint == 1) {
      point = Setpoints.blueFirstGridLeftCone;
    }
    if (m_poseSetpoint == 2) {
      point = Setpoints.blueFirstGridCube;
    }
    if (m_poseSetpoint == 3) {
      point = Setpoints.blueFirstGridRightCone;
    }
    if (m_poseSetpoint == 4) {
      point = Setpoints.blueSecondGridLeftCone;
    }
    if (m_poseSetpoint == 5) {
      point = Setpoints.blueSecondGridCube;
    }
    if (m_poseSetpoint == 6) {
      point = Setpoints.blueSecondGridRightCone;
    }
    if (m_poseSetpoint == 7) {
      point = Setpoints.blueThirdGridLeftCone;
    }
    if (m_poseSetpoint == 8) {
      point = Setpoints.blueThirdGridCube;
    }
    if (m_poseSetpoint == 9) {
      point = Setpoints.blueThirdGridRightCone;
    }

    if (alliance.equals(Alliance.Blue)) {
      return point;
    } else {
      return point.flipPathPoint();
    }
  }

  public void update() {
    elevatorYMeters = m_elevator.getPositionYMeters();
    elevatorXMeters = m_elevator.getPositionXMeters();
    wristAngleRotation2d = m_wrist.getAngle();
    Rotation2d wristAngleDesired = m_wrist.m_desiredAngle;
    realWantedWristRotation2d = m_wrist.userWantedAngle;
    // checkIfWristBreak(elevatorXMeters, wristAngleDesired);
    // checkIfBreakElevator();
    Pose3d armPosition = getArmPosition(m_drive.getPose(), m_elevator.getPositionXMeters(),
        m_elevator.getPositionYMeters(), wristAngleRotation2d,
        new Transform3d(new Translation3d(ElevatorConstants.kCarriageArmLength, 0, 0),
            new Rotation3d(0, wristAngleRotation2d.getRadians(), 0)),
        new Translation3d(Units.inchesToMeters(16), 0, 0));

    Mechanism2d fullMech = new Mechanism2d(2, 2);
    fullMech.getRoot("ElevatorBottom", 0, ElevatorConstants.kMinHeightMeters)
        .append(new MechanismLigament2d("Elevator", m_elevator.getTravelDistanceMeters(),
            ElevatorConstants.kAngle.getDegrees()))
        .append(new MechanismLigament2d("Arm", Constants.ElevatorConstants.kCarriageArmLength,
            0 - ElevatorConstants.kAngle.getDegrees()))
        .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12),
            -wristAngleRotation2d.getDegrees()));

    Logger.getInstance().recordOutput("RobotState/Elevator", fullMech);
    Logger.getInstance().recordOutput("RobotState/ElevatorSpot", m_elevatorPosition);
    Logger.getInstance().recordOutput("RobotState/IntakeSpot", m_intakePosition);
    Logger.getInstance().recordOutput("RobotState/ArmSpot", armPosition);
    SmartDashboard.putNumber("Pose Setpoint", m_poseSetpoint);

  }

  public void checkIfBreakElevator() {
    if (m_intakePosition == null) {
      System.out.println("null pos");
      return;
    }
    // System.out.println(m_elevator.getDesiredMeters() < m_elevator.getCurrentHeightMeters());
    if (fieldGeomUtil.overConesOrCubes(m_intakePosition)
        && (m_elevator.getDesiredMeters() + Units.inchesToMeters(3) < m_elevator.getCurrentHeightMeters())) {
      m_elevator.setHeightCommand(m_elevator.getPositionYMeters()).schedule();
    }
  }

  public Pose3d getArmPosition(Pose2d robotPose, double elevatorXMeters, double elevatorYMeters,
      Rotation2d wristAngleRotation2d, Transform3d armLength, Translation3d intake) {
    Pose3d pose3d = new Pose3d(robotPose);
    m_elevatorPosition = pose3d
        .plus(new Transform3d(new Translation3d(elevatorXMeters, 0, elevatorYMeters), new Rotation3d()));
    m_armPosition = m_elevatorPosition.plus(armLength);
    m_intakePosition = m_armPosition.plus(new Transform3d(intake, new Rotation3d()));
    // m_intakePosition.transformBy(new Transform3d(new Translation3d(), new Rotation3d(wristAngleRotation2d.getRadians(), 0, 0)));

    return m_intakePosition;

  }

  public void checkIfWristBreak(double elevatorXMeters, Rotation2d wristAngleDesired) {
    if (Units.metersToInches(elevatorYMeters) < 8 && wristAngleDesired.getDegrees() < 0) {
      m_wrist.setAngleToNotBreak(Rotation2d.fromDegrees(0));
    } else {
      m_wrist.setAngleToNotBreak(realWantedWristRotation2d);
    }
  }

  public double getMorphedVelocityMultiplier() {
    double travelPercent = MathUtil.clamp(m_elevator.getTravelDistanceMeters() / ElevatorConstants.kMaxTravelMeters, 0,
        1);
    return Math.sqrt(1 - travelPercent);
  }

  public Command setpointCommand(double[] setpoint) {
    return setpointCommand(Setpoint.fromArray(setpoint));
  }

  public Command setpointCommand(Setpoint setpoint) {
    return Commands.parallel(
        m_elevator.setHeightCommand(setpoint.heightMeters),
        m_wrist.setAngleCommand(setpoint.angle));
  }
}

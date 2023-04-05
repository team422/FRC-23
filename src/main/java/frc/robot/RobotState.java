package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathplanner.ExtendedPathPoint;
import frc.lib.utils.FieldGeomUtil;
import frc.lib.utils.FullDesiredRobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class RobotState {

  private static RobotState instance;
  public Drive m_drive;
  public Intake m_intake;
  public Elevator m_elevator;
  public Wrist m_wrist;

  public Pose3d m_armPosition = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public Pose3d m_wristPosition = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public Pose3d m_intakePosition = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public Pose3d m_elevatorPosition = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

  public double elevatorYMeters;
  public double elevatorXMeters;
  public Rotation2d wristAngleRotation2d;
  public Rotation2d realWantedWristRotation2d;
  public FieldGeomUtil fieldGeomUtil = new FieldGeomUtil();
  public int m_poseSetpoint;

  public int m_grid = 1;
  public int m_column = 1;
  public int m_height = 2;

  public String m_scoringSetpoint = "blueFirstGridLeftHigh";

  public Pose3d m_robotPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public double m_lastCameraTimestamp = -10;
  public Pose3d m_robotPoseLowConfidence = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public double m_lastCameraTimestampLowConfidence = -10;

  public FullDesiredRobotState m_scoringPose = new FullDesiredRobotState(new Pose2d(), 0,
      Rotation2d.fromDegrees(0));
  public double m_elevatorHeightMeters = 0;

  private RobotState(Drive drive, Intake intake, Elevator elevator, Wrist wrist) {
    m_drive = drive;
    m_intake = intake;
    m_elevator = elevator;
    m_wrist = wrist;
    m_poseSetpoint = 5;
    m_elevatorHeightMeters = m_elevator.getCurrentHeightMeters();
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

  public Pose3d get3dPosition() {
    return m_robotPose;

  }

  public Pose3d getCamPositionLowConfidence() {
    return m_robotPoseLowConfidence;
  }

  public void setCamPositionLowConfidence(Pose3d pose) {
    m_robotPoseLowConfidence = pose;
    m_lastCameraTimestampLowConfidence = Timer.getFPGATimestamp();
    Logger.getInstance().recordOutput("Drive/VisionLowConfidence", pose.toPose2d());
  }

  public void set3dPosition(Pose3d pose) {
    m_robotPose = pose;
    m_lastCameraTimestamp = Timer.getFPGATimestamp();
    Logger.getInstance().recordOutput("Drive/VisionHighConfidence", pose.toPose2d());
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
    Pose3d armPosition = new Pose3d();
    // if (RobotConstants.AScopeLogging) {
    armPosition = getArmPosition(m_drive.getPose(), m_elevator.getPositionXMeters(),
        m_elevator.getPositionYMeters(), wristAngleRotation2d,
        new Transform3d(new Translation3d(ElevatorConstants.kCarriageArmLength, 0, 0),
            new Rotation3d(0, wristAngleRotation2d.getRadians(), 0)),
        new Translation3d(Units.inchesToMeters(16), 0, 0));
    // }
    // Mechanism2d fullMech = new Mechanism2d(2, 2);
    // fullMech.getRoot("ElevatorBottom", 0, ElevatorConstants.kMinHeightMeters)
    //     .append(new MechanismLigament2d("Elevator", m_elevator.getTravelDistanceMeters(),
    //         ElevatorConstants.kAngle.getDegrees()))
    //     .append(new MechanismLigament2d("Arm", Constants.ElevatorConstants.kCarriageArmLength,
    //         0 - ElevatorConstants.kAngle.getDegrees()))
    //     .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12),
    //         -wristAngleRotation2d.getDegrees()));

    // Logger.getInstance().recordOutput("RobotState/Elevator", fullMech);
    Logger.getInstance().recordOutput("RobotState/ElevatorSpot", m_elevatorPosition);
    Logger.getInstance().recordOutput("RobotState/IntakeSpot", m_intakePosition);
    Logger.getInstance().recordOutput("RobotState/ArmSpot", armPosition);
    if (Robot.isSimulation()) {
      setCamPositionLowConfidence(new Pose3d(m_drive.getPose()));
    }
    // SmartDashboard.putNumber("Pose Setpoint", m_poseSetpoint);

  }

  public void checkIfBreakElevator() {
    if (m_intakePosition == null) {
      System.out.println("null pos");
      return;
    }
    // System.out.println(m_elevator.getDesiredMeters() < m_elevator.getCurrentHeightMeters());
    // if (fieldGeomUtil.overConesOrCubes(m_intakePosition)
    //     && (m_elevator.getDesiredMeters() + Units.inchesToMeters(3) < m_elevator.getCurrentHeightMeters())) {
    //   m_elevator.setHeightCommand(m_elevator.getPositionYMeters()).schedule();
    // }
  }

  public Command setGridCommand(int grid) {
    return Commands.runOnce(() -> {
      this.setGrid(grid);
      System.out.print("grid set to " + grid);
    });
  }

  public void setGrid(int grid) {
    m_grid = grid;
    this.updateScoringPosition();
  }

  public Command setColumnCommand(int column) {
    return Commands.runOnce(() -> {
      this.setColumn(column);
    });
  }

  public void setColumn(int column) {
    m_column = column;
    this.updateScoringPosition();
  }

  public Command setHeightCommand(int height) {
    return Commands.runOnce(() -> {
      this.setHeight(height);
    });
  }

  public void setHeight(int height) {
    m_height = height;
    this.updateScoringPosition();
  }

  public void updateScoringPosition() {
    String fin = "blue";
    if (m_grid == 1) {
      fin += "First";
    } else if (m_grid == 2) {
      fin += "Second";
    } else if (m_grid == 3) {
      fin += "Third";
    }
    fin += "Grid";
    if (m_column == 1) {
      fin += "Left";
    } else if (m_column == 2) {
      fin += "Cube";
    } else if (m_column == 3) {
      fin += "Right";
    }
    if (m_height == 2) {
      fin += "Mid";
    } else if (m_height == 3) {
      fin += "High";
    }

    m_scoringSetpoint = fin;
    // System.out.println("scoring setpoint set to " + fin);
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

  public Command setpointCommandSequential(double[] setpoint) {
    return Commands.sequence(
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(75)),
        m_elevator.testSetHeightCommand(setpoint[0], Units.inchesToMeters(46)),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(setpoint[1])));
  }

  public Command setpointCommandParallel(double[] setpoint) {
    return Commands.parallel(
        m_elevator.setHeightCommand(setpoint[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(setpoint[1])));
  }

  public Command safeBack(double[] setpoint) {
    return m_wrist.setAngleCommand(Rotation2d.fromDegrees(setpoint[1])).until(() -> {
      return m_wrist.atSetpoint();
    }).andThen();
  }

  public Command autoScore(Supplier<Pose3d> intakeEndPoseSup, Supplier<Rotation2d> intakeAngleSup,
      Supplier<Double> distanceToIntakeSup) {
    // Pose3d intakeEndPose = intakeEndPoseSup.get();
    // Rotation2d intakeAngle = intakeAngleSup.get();
    // double distanceToIntake = distanceToIntakeSup.get();
    // System.out.println(intakeEndPose);
    // System.out.println(intakeAngle);
    // System.out.println(distanceToIntake);
    return Commands.sequence(
        Commands.runOnce(
            () -> RobotState.getInstance().getClosestPoseToSetpoint(intakeEndPoseSup.get(), intakeAngleSup.get(),
                distanceToIntakeSup.get())),
        Commands.parallel(
            new DriveToPoint(m_drive,
                () -> RobotState.getInstance()
                    .getScoringSetpoint()
                    //.getClosestPoseToSetpoint(intakeEndPoseSup.get(), intakeAngleSup.get(), distanceToIntakeSup.get())
                    .getPose(),
                Constants.DriveConstants.holonomicDrive,
                () -> 0.0,
                () -> 0.0, () -> 0.0),
            Commands.runOnce(() -> m_elevator.setHeight(RobotState.getInstance()
                .getScoringSetpoint()
                .getHeightOfElevator())),
            Commands.runOnce(() -> m_wrist.setAngle(RobotState.getInstance()
                .getScoringSetpoint()
                .getAngleOfWrist()))));

  }

  public FullDesiredRobotState getClosestPoseToSetpoint(Pose3d intakeEndPose, Rotation2d intakeAngle,
      double distanceToIntake) {
    // find x distance of everything added together and then find the closest one
    double intakeLength = intakeAngle.getCos() * IntakeConstants.intakeLengthMeters;
    double intakeHeight = intakeAngle.getSin() * IntakeConstants.intakeLengthMeters * -1;
    double armLength = ElevatorConstants.kCarriageArmLength;
    double elevatorWantedHeight = intakeEndPose.getZ() + intakeHeight;
    double elevatorXMeters = m_elevator.getPositionXMetersAtHeight(elevatorWantedHeight);
    double xDistance = intakeLength + armLength + elevatorXMeters
        - Constants.ElevatorConstants.kDistanceToCenterOfRobot + distanceToIntake;
    // System.out.println(
    //     "intkae length:" + intakeLength +
    //         " intake Height " + intakeHeight +
    //         " Arm Length " + armLength +
    //         " elevatorWantedHeight " + elevatorWantedHeight +
    //         " elevatorXMeters " + elevatorXMeters +
    //         "x final distance" + xDistance);

    // System.out.println("desired pose: " + intakeEndPose);
    // for the sake of not overrunning the loop, we only check a setpoint every 10 degrees starting from x + distance 
    ArrayList<Pose2d> poses = new ArrayList<>();
    Rotation2d angle = Rotation2d.fromDegrees(0);
    double x = xDistance * angle.getCos();
    double y = xDistance * angle.getSin();
    intakeEndPose.toPose2d().plus(new Transform2d(new Translation2d(x, y), new Rotation2d()));
    poses.add(
        intakeEndPose.toPose2d()
            .plus(new Transform2d(new Translation2d(x, y), angle.plus(Rotation2d.fromDegrees(180)))));
    double minDistance = Double.MAX_VALUE;

    Pose2d closestPose = new Pose2d();
    for (Pose2d pose : poses) {
      double distance = pose.getTranslation().getDistance(m_drive.getPose().getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = pose;
      }
    }
    // int i = 0;
    // for (Pose2d pose : poses) {
    //   Logger.getInstance().recordOutput("RobotState/ClosestPose: " + i, pose);
    //   i++;
    // }
    Logger.getInstance().recordOutput("RobotState/BestClosestPose", closestPose);

    // System.out.println("Closest Pose: " + closestPose);
    // System.out.println("height of elevator: " + elevatorWantedHeight);
    FullDesiredRobotState desiredRobotState = new FullDesiredRobotState(closestPose, elevatorWantedHeight,
        intakeAngle);
    RobotState.getInstance().setScoringSetpoint(desiredRobotState);
    return desiredRobotState;
  }

  public void setScoringSetpoint(FullDesiredRobotState desiredRobotState) {
    this.m_scoringPose = desiredRobotState;
  }

  public FullDesiredRobotState getScoringSetpoint() {
    System.out.println("scoring pose: " + this.m_scoringPose);
    return this.m_scoringPose;
  }

  public String getFullScoringString() {
    return m_scoringSetpoint;
  }

  public Rotation2d getWristPosition() {
    if (m_scoringSetpoint.toLowerCase().contains("cube") && m_scoringSetpoint.toLowerCase().contains("high")) {
      System.out.println("cube high");
      return Constants.Setpoints.kIntakeApproachAngleHighCube;

    } else if (m_scoringSetpoint.toLowerCase().contains("cube") && m_scoringSetpoint.toLowerCase().contains("mid")) {
      System.out.println("cube mid");
      return Constants.Setpoints.kIntakeApproachAngleMidCube;
    } else if (m_scoringSetpoint.toLowerCase().contains("high")) {
      System.out.println("cone high");
      return Setpoints.kIntakeApproachAngleHighCone;

    } else {
      System.out.println("cone mid");
      return Setpoints.kIntakeApproachAngleMidCone;
    }
  }

  public Pose3d getScoringPose() {
    return fieldGeomUtil.getClosestScoringPose(m_drive.getPose(), m_height);
    // return fieldGeomUtil.getClosestScoringPose();
  }

  public void setClosestScoringPoseName(String name) {
    m_scoringSetpoint = name;
  }

}

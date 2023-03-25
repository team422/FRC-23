package frc.robot.commands.autonomous;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class AutoFactory extends CommandBase {
  public static final PIDConstants linearPIDConstants = new PIDConstants(10, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(4, 0, 0);

  private final Drive m_drive;
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Intake m_intake;

  private final Map<String, Command> m_eventMap;

  public AutoFactory(Drive drive, Elevator elevator, Wrist wrist, Intake intake) {
    m_drive = drive;
    m_elevator = elevator;
    m_wrist = wrist;
    m_intake = intake;

    // Define PathPlanner Event Map
    Command stow = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.stowVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.stowVerticalCommandSetpoints[1])),
        Commands.print("stow"));
    Command coneHigh = Commands.sequence(
        m_elevator.setHeightCommand(Setpoints.coneHighCommandSetpoints[0]),
        Commands.waitSeconds(0.1),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.coneHighCommandSetpoints[1])),
        Commands.print("coneHighElevator"));
    // Command coneHigh = RobotState.getInstance().setpointCommandCone(null)
    Command cubeHigh = Commands.sequence(
        m_elevator.setHeightCommand(Setpoints.cubeHighCommandSetpoints[0]),
        Commands.waitSeconds(.2),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.cubeHighCommandSetpoints[1])),
        Commands.print("coneHighElevator"));

    Command autoConeHigh = Commands.sequence(
        Commands.parallel(
            m_elevator.setHeightCommand(Setpoints.coneHighCommandSetpoints[0]),
            m_wrist.setAngleCommand(Rotation2d.fromDegrees(85))),
        Commands.waitSeconds(0.5),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.coneHighCommandSetpoints[1])));

    Command cubeGround = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.pickUpCubeGroundCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.pickUpCubeGroundCommandSetpoints[1] - 5)),
        m_intake.setDesiredSpeedCommand(0),
        Commands.print("cubeGround"));
    Command coneGround = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.pickUpConeGroundCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.pickUpConeGroundCommandSetpoints[1] - 3)),
        Commands.print("coneground"));
    Command coneVertical = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.pickUpConeVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.pickUpConeVerticalCommandSetpoints[1])),
        Commands.print("coneVertical"));
    Command cubeGroundBump = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.pickUpCubeGroundCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.pickUpCubeGroundCommandSetpoints[1])),
        m_intake.setDesiredSpeedCommand(.5),

        Commands.print("cubeGround"));
    Command balanceStation = new ChargeStationBalance(m_drive);
    Command zeroHeading = new ZeroHeading(m_drive);
    Command coneDrop = m_intake.setDesiredSpeedCommand(0.5);
    Command conePickup = m_intake.setDesiredSpeedCommand(-0.5);
    Command cubeDrop = m_intake.setDesiredSpeedCommand(-1);
    Command cubePickup = m_intake.setDesiredSpeedCommand(0.5);
    Command stopIntake = m_intake.setDesiredSpeedCommand(0.0);
    m_eventMap = Map.ofEntries(
        Map.entry("setpointStow", stow),
        Map.entry("setpointCubeGround", cubeGround),
        Map.entry("setpointCubeGroundBump", cubeGroundBump),
        Map.entry("setpointConeHigh", coneHigh),
        Map.entry("setpointCubeHigh", cubeHigh),
        Map.entry("setpointConeGround", coneGround),
        Map.entry("intakeCubeIn", cubePickup),
        Map.entry("setpointConeVertical", coneVertical),
        Map.entry("intakeCubeOut", cubeDrop),
        Map.entry("intakeConeIn", conePickup),
        Map.entry("intakeConeOut", coneDrop),
        Map.entry("wait", Commands.waitSeconds(.2)),
        Map.entry("intakeStop", stopIntake),
        Map.entry("balance", balanceStation),
        Map.entry("zeroHeading", zeroHeading),
        Map.entry("setpointConeHighWait", autoConeHigh));
    // m_eventMap = Map.ofEntries(
    //     Map.entry("a", Commands.print("aaaaaaaaaaaaa a")),
    //     Map.entry("stow", stow),
    //     Map.entry("coneHighElevator", coneHighElevator),
    //     Map.entry("coneHighWrist", coneHighWrist),
    //     Map.entry("cubeGround", cubeGround),
    //     Map.entry("intakeCubeIn", cubePickup),
    //     Map.entry("cubeOut", cubeDrop),
    //     Map.entry("stopIntake", stopIntake),
    //     Map.entry("brake", m_drive.brakeCommand()),
    //     Map.entry("pickUpConeLow",
    //         m_elevator.setHeightCommand(SetpointConstants.pickUpConeVerticalCommandSetpoints[0])),
    //     Map.entry("elevatorHigh", m_elevator.setHeightCommand(Units.inchesToMeters(5))),
    //     Map.entry("charge", new ChargeStationBalance(drive)));

    if (Constants.MetaConstants.pathTuningMode) {
      PathPlannerServer.startServer(5811);
    }
  }

  public List<PathPlannerTrajectory> loadPathGroupByName(String name) {
    return PathPlanner.loadPathGroup(
        name,
        DriveConstants.kMaxSpeedMetersPerSecondAuto,
        DriveConstants.kMaxAccelMetersPerSecondSqAuto);
  }

  public CommandBase getAutoCommand(String name) {
    return getAutoCommand(loadPathGroupByName(name));
  }

  private CommandBase getAutoCommand(List<PathPlannerTrajectory> paths) {
    // Create Auto builder
    BaseAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_drive::getPose,
        m_drive::resetPose,
        linearPIDConstants,
        angularPIDConstants,
        m_drive::drive,
        m_eventMap,
        true, // TODO: ENABLE (AND TEST) BEFORE COMP
        m_drive);

    return autoBuilder.fullAuto(paths).andThen(m_drive.brakeCommand());
  }
}

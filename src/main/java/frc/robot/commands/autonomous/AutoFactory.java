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
        m_elevator.setHeightCommand(Setpoints.stow.heightMeters),
        m_wrist.setAngleCommand(Setpoints.stow.angle),
        m_intake.setDesiredSpeedCommand(0),
        Commands.print("stow"));

    Command coneHigh = Commands.sequence(
        m_intake.holdConeCommand(),
        m_elevator.setHeightCommand(Setpoints.coneHigh.heightMeters),
        Commands.waitSeconds(0.5),
        m_wrist.setAngleCommand(Setpoints.coneHigh.angle),
        Commands.print("coneHighElevator"));

    Command cubeHigh = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.cubeHigh.heightMeters),
        m_wrist.setAngleCommand(Setpoints.cubeHigh.angle),
        Commands.print("coneHighElevator"));

    Command autoConeHigh = Commands.sequence(
        Commands.parallel(
            m_elevator.setHeightCommand(Setpoints.coneHigh.heightMeters),
            m_wrist.setAngleCommand(Rotation2d.fromDegrees(85))),
        Commands.waitSeconds(0.3),
        m_wrist.setAngleCommand(Setpoints.coneHigh.angle));

    Command cubeGround = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.kIntakeGroundCube.heightMeters),
        m_wrist.setAngleCommand(Setpoints.kIntakeGroundCube.angle),
        m_intake.setDesiredSpeedCommand(0),
        Commands.print("cubeGround"));

    Command balanceStation = new ChargeStationBalance(m_drive);
    Command zeroHeading = new ZeroHeading(m_drive);

    m_eventMap = Map.ofEntries(
        Map.entry("setpointStow", stow),
        Map.entry("setpointCubeGround", cubeGround),
        Map.entry("setpointCubeGroundBump", cubeGround),
        Map.entry("setpointConeHigh", coneHigh),
        Map.entry("setpointCubeHigh", cubeHigh),
        Map.entry("intakeCubeIn", m_intake.intakeCubeCommand()),
        Map.entry("intakeCubeOut", m_intake.dropCubeCommand()),
        Map.entry("intakeConeIn", m_intake.intakeConeCommand()),
        Map.entry("intakeConeOut", m_intake.dropConeCommand()),
        Map.entry("intakeStop", m_intake.stopCommand()),
        Map.entry("wait", Commands.waitSeconds(0.35)),
        Map.entry("balance", balanceStation),
        Map.entry("zeroHeading", zeroHeading),
        Map.entry("setpointConeHighWait", autoConeHigh));

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

package frc.robot.commands.autonomous;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.AprilTagCamera;

public class AutoFactory extends CommandBase {
  public static final PIDConstants linearPIDConstants = new PIDConstants(6, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(2, 0, 0);

  private final Drive m_drive;
  private final AprilTagCamera m_camera;
  private final Elevator m_elevator;

  private final Map<String, Command> m_eventMap;

  public AutoFactory(Drive drive, AprilTagCamera camera, Elevator elevator) {
    m_drive = drive;
    m_camera = camera;
    m_elevator = elevator;

    // Define PathPlanner Event Map
    m_eventMap = Map.of(
        "a", Commands.print("a"),
        "b", Commands.print("b"),
        "c", Commands.print("c"),
        "d", Commands.print("d"),
        "brake", m_drive.brakeCommand(),
        "elevatorLow", m_elevator.setPositionCommand(Units.feetToMeters(1.5)),
        "elevatorHigh", m_elevator.setPositionCommand(Units.feetToMeters(5)),
        "charge", ChargeStationBalance.charge(drive));

    if (Constants.kDebugMode) {
      PathPlannerServer.startServer(5811);
    }
  }

  public List<PathPlannerTrajectory> loadPathGroupByName(String name) {
    return PathPlanner.loadPathGroup(
        name,
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelMetersPerSecondSq);
  }

  public CommandBase getAutoCommand(String name) {
    return getAutoCommand(loadPathGroupByName(name));
  }

  private CommandBase getAutoCommand(List<PathPlannerTrajectory> paths) {
    // Create Auto builder
    BaseAutoBuilder autoBuilder = new SwerveAutoBuilder(
        m_drive::getPose,
        m_drive::resetPose,
        linearPIDConstants, angularPIDConstants,
        m_drive::drive,
        m_eventMap,
        true, // TODO: ENABLE (AND TEST) BEFORE COMP
        m_drive);

    return autoBuilder.fullAuto(paths).andThen(m_drive.hardBrakeCommand());
  }
}

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gyro.GyroSub;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

public class AutoFactory extends CommandBase {
  public static final PIDConstants linearPIDConstants = new PIDConstants(6, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(2, 0, 0);

  private final Drive m_drive;
  private final Elevator m_elevator;
  private final GyroSub m_gyro;
  private final Wrist m_wrist;
  private final Intake m_intake;

  private final Map<String, Command> m_eventMap;

  public AutoFactory(Drive drive, Elevator elevator, Wrist wrist, Intake intake, GyroSub gyro) {
    m_drive = drive;
    m_elevator = elevator;
    m_gyro = gyro;
    m_wrist = wrist;
    m_intake = intake;

    // Define PathPlanner Event Map
    Command stow = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.stowVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.stowVerticalCommandSetpoints[1])),
        m_intake.setDesiredSpeedCommand(0));
    Command coneHighElevator = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.coneHighCommandSetpoints[0]),
        m_intake.setDesiredSpeedCommand(0));
    Command coneHighWrist = Commands.parallel(
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.coneHighCommandSetpoints[1])),
        m_intake.setDesiredSpeedCommand(0));
    Command coneDrop = m_intake.setDesiredSpeedCommand(0.5);
    Command conePickup = m_intake.setDesiredSpeedCommand(-0.5);
    Command cubeDrop = m_intake.setDesiredSpeedCommand(-0.5);
    Command cubePickup = m_intake.setDesiredSpeedCommand(0.5);

    m_eventMap = Map.of(
        "stow", stow,
        "coneHighElevator", coneHighElevator,
        "coneHighWrist", coneHighWrist,
        "d", Commands.print("d"),
        "brake", m_drive.brakeCommand(),
        "pickUpConeLow", m_elevator.setHeightCommand(SetpointConstants.pickUpConeVerticalCommandSetpoints[0]),
        "elevatorHigh", m_elevator.setHeightCommand(Units.inchesToMeters(5)),
        "charge", new ChargeStationBalance(drive, m_gyro));

    if (Constants.MetaConstants.pathTuningMode) {
      PathPlannerServer.startServer(5811);
    }
  }

  public List<PathPlannerTrajectory> loadPathGroupByName(String name) {
    return PathPlanner.loadPathGroup(
        name,
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxSpeedMetersPerSecond);
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
        false, // TODO: ENABLE (AND TEST) BEFORE COMP
        m_drive);

    return autoBuilder.fullAuto(paths).andThen(m_drive.brakeCommand());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.SimVisionSystem;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.accelerometer.Pigeon2Accelerometer;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.lib.utils.FieldUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.SingleUserXboxControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOSim;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOWPI;
import frc.robot.subsystems.drive.gyro.PigeonIO;
import frc.robot.subsystems.drive.module.SwerveModuleIOMK2Neo;
import frc.robot.subsystems.drive.module.SwerveModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.AprilTagCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive m_drive;
  private Elevator m_elevator;

  private Mechanism2d m_mechanism;
  private AprilTagCamera m_camera;

  private AprilTagFieldLayout m_tagLayout;

  private AutoFactory m_autoFactory;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureAprilTags();
    configureAllianceSettings(DriverStation.getAlliance());
    configureSubsystems();
    configureButtonBindings();
    configureAuto();
  }

  private void configureAuto() {
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoFactory = new AutoFactory(m_drive, m_camera, m_elevator);

    // Add basic autonomous commands
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addOption("Zero Absolute Encoders", DebugCommands.zeroTurnAbsoluteEncoders(m_drive));

    // Add PathPlanner Auto Commands
    PathPlannerUtil.getExistingPaths().forEach(path -> {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    });
  }

  private void configureAprilTags() {
    try {
      m_tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

      if (Robot.isSimulation()) {
        m_simVisionSystem = Optional.of(
            new SimVisionSystem(
                VisionConstants.kAprilTagCameraGrayConfig.getName(),
                VisionConstants.kAprilTagCameraGrayConfig.getFOV(),
                VisionConstants.kAprilTagCameraGrayConfig.getRobotToCamera(),
                5,
                640, 480,
                10.0));
      }
    } catch (IOException e) {
      throw new RuntimeException("Unable to load apriltag field layout!");
    }
  }

  public void configureAllianceSettings(Alliance alliance) {
    var origin = alliance == Alliance.Red
        ? OriginPosition.kRedAllianceWallRightSide
        : OriginPosition.kBlueAllianceWallRightSide;
    m_tagLayout.setOrigin(origin);

    FieldUtil.getDefaultField().setObjectGlobalPoses("AprilTags",
        m_tagLayout.getTags().stream().map(x -> x.pose.toPose2d()).toArray(Pose2d[]::new));

    m_simVisionSystem.ifPresent(visionSystem -> {
      visionSystem.clearVisionTargets();
      visionSystem.addVisionTargets(m_tagLayout);
    });
  }

  private void configureSubsystems() {
    m_mechanism = new Mechanism2d(70, 70);

    var root = m_mechanism.getRoot("robot", 5, 5);
    var elevatorLigament = root.append(new MechanismLigament2d("elevator", 0, Elevator.kElevatorAngle.getDegrees()));
    var staticArmLigament = elevatorLigament
        .append(new MechanismLigament2d("staticArm", 16, -Elevator.kElevatorAngle.getDegrees(), 10,
            new Color8Bit(Color.kBlue)));
    var wristLigament = staticArmLigament
        .append(new MechanismLigament2d("wrist", 4, 0, 10, new Color8Bit(Color.kAliceBlue)));

    WPI_Pigeon2 pigeon = new WPI_Pigeon2(ElectricalConstants.kGyroPort);

    if (Robot.isReal()) {
      pigeon.configMountPose(AxisDirection.NegativeX, AxisDirection.PositiveZ);
      m_drive = new Drive(
          // new GyroIOWPIWrapper(new ADXRS450_Gyro()),
          new PigeonIO(pigeon),
          new AccelerometerIOWPI(new Pigeon2Accelerometer(pigeon)),
          new SwerveModuleIOMK2Neo(
              ElectricalConstants.kFrontLeftTurnMotorPort,
              ElectricalConstants.kFrontLeftDriveMotorPort,
              ElectricalConstants.kFrontLeftCANCoderPort,
              Rotation2d.fromRadians(0.618)),
          new SwerveModuleIOMK2Neo(
              ElectricalConstants.kFrontRightTurnMotorPort,
              ElectricalConstants.kFrontRightDriveMotorPort,
              ElectricalConstants.kFrontRightCANCoderPort,
              Rotation2d.fromRadians(2.966)),
          new SwerveModuleIOMK2Neo(
              ElectricalConstants.kBackLeftTurnMotorPort,
              ElectricalConstants.kBackLeftDriveMotorPort,
              ElectricalConstants.kBackLeftCANCoderPort,
              Rotation2d.fromRadians(0.548)),
          new SwerveModuleIOMK2Neo(
              ElectricalConstants.kBackRightTurnMotorPort,
              ElectricalConstants.kBackRightDriveMotorPort,
              ElectricalConstants.kBackRightCANCoderPort,
              Rotation2d.fromRadians(0.993)));
      m_elevator = new Elevator(new ElevatorIOSim(), elevatorLigament);
      // m_elevator = new Elevator(new ElevatorIONeo(
      //     ElectricalConstants.kElevatorLeaderPort,
      //     ElectricalConstants.kElevatorFollowerPort), elevatorLigament);
    } else {
      m_drive = new Drive(
          new PigeonIO(pigeon),
          new AccelerometerIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
      m_elevator = new Elevator(new ElevatorIOSim(), elevatorLigament);
    }

    SmartDashboard.putData("Robot Mechanism", m_mechanism);
    m_camera = new AprilTagCamera(
        VisionConstants.kAprilTagCameraGrayConfig.getName(),
        VisionConstants.kAprilTagCameraGrayConfig.getRobotToCamera(),
        m_tagLayout,
        m_drive.getPoseEstimator());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var singleUserControls = new SingleUserXboxControls(5);
    DriverControls driverControls = singleUserControls;
    OperatorControls operatorControls = singleUserControls;

    // DriverControls driverControls = new DualJoystickDriverControls(0, 1);
    // OperatorControls operatorControls = new XboxOperatorControls(5);

    DriveCommandConfig driveConfig = new DriveCommandConfig(
        DriveConstants.kDriveKinematics,
        m_drive::getPose,
        m_drive::getModuleStates,
        m_drive.getGyro()::getRate,
        m_drive::drive,
        m_drive);

    BaseDriveCommand fieldRelativeDrive = DriveCommands.fieldRelativeDrive(driveConfig, driverControls);
    BaseDriveCommand robotRelativeDrive = DriveCommands.robotRelativeDrive(driveConfig, driverControls);
    BaseDriveCommand joystickAngleDrive = DriveCommands.joystickAngleDrive(driveConfig, driverControls);

    m_drive.setDefaultCommand(fieldRelativeDrive);
    driverControls.robotRelativeDrive().whileTrue(robotRelativeDrive);
    driverControls.joystickAngleDrive().whileTrue(joystickAngleDrive);

    // driverControls.testButton().onTrue(ChargeStationBalance.charge(m_drive));
    driverControls.resetPose().onTrue(m_drive.resetPoseCommand(new Pose2d(5, 3, Rotation2d.fromDegrees(0))));
    driverControls.resetPoseToVisionEst().onTrue(m_drive.resetPoseCommand(m_camera::getLatestEstimatedPose));

    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
    operatorControls.zeroTurnAbsoluteEncoders().onTrue(DebugCommands.zeroTurnAbsoluteEncoders(m_drive));

    // Elevator Buttons
    operatorControls.setElevatorPositionHigh().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(5)));
    operatorControls.setElevatorPositionMid().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(3)));
    operatorControls.setElevatorPositionLow().onTrue(m_elevator.setPositionCommand(Units.feetToMeters(1.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Robot.isSimulation()) {
      String selectedAuto = m_autoChooser.getSendableChooser().getSelected();
      if (PathPlannerUtil.getExistingPaths().contains(selectedAuto)) {
        System.out.println("Reloading pathplanner path file: " + selectedAuto);
        return m_autoFactory.getAutoCommand(selectedAuto);
      }
    }
    return m_autoChooser.get();
  }

  //#region Sim Stuff

  private Optional<SimVisionSystem> m_simVisionSystem = Optional.empty();

  public void simulationPeriodic() {
    m_simVisionSystem.ifPresent(visionSystem -> {
      visionSystem.processFrame(m_drive.getPose());
    });
  }

  //#endregion

  //#region Robot State Callbacks

  public void periodic() {
    Logger.getInstance().recordOutput("Mechanism2d", m_mechanism);
  }

  public void onDisabled() {
    if (Constants.kDebugMode) {
      DebugCommands.brakeAndReset(m_drive).schedule();
    }
  }

  //#endregion
}

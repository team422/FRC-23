// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.drive.DriveThroughPointsToLoadingStation;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.commands.drive.TeloepDrive;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIONeo;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONeo550;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.CameraAprilTag;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOThroughBoreSparkMaxAlternate;

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
  private Intake m_intake;
  private CameraAprilTag[] m_cams;
  private Wrist m_wrist;
  private Elevator m_elevator;
  private CANSparkMax m_throughboreSparkMaxIntakeMotor;
  private RobotState m_robotState;
  private AutoFactory m_autoFactory;
  private AprilTagFieldLayout m_layout;
  private LED m_LED;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureAprilTags();
    configureSubsystems();
    configureButtonBindings();
    configureAuto();
  }

  public void configureAprilTags() {
    try {
      m_layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      System.out.println("AprilTag field layout not found:" + e);
    }
  }

  private void configureAllianceSettings() {
    var origin = DriverStation.getAlliance() == Alliance.Blue
        ? OriginPosition.kBlueAllianceWallRightSide
        : OriginPosition.kRedAllianceWallRightSide;
    m_layout.setOrigin(origin);
  }

  private void configureAuto() {
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoFactory = new AutoFactory(m_drive, m_elevator, m_wrist, m_intake);

    // Add basic autonomous commands
    // m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
    m_autoChooser.addDefaultOption("Top Cone Cube Balance", Commands.none());

    // Add PathPlanner Auto Commands
    PathPlannerUtil.getExistingPaths().forEach(path -> {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    });
  }

  private void configureSubsystems() {
    if (Robot.isReal()) {
      SwerveModuleIO[] m_swerveModuleIOs = {
          new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftFrontDrivingMotorPort, Ports.leftFrontTurningMotorPort,
              Ports.leftFrontCanCoderPort),
          new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightFrontDriveMotorPort, Ports.rightFrontTurningMotorPort,
              Ports.rightFrontCanCoderPort),
          new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftRearDriveMotorPort, Ports.leftRearTurningMotorPort,
              Ports.leftRearCanCoderPort),
          new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightRearDriveMotorPort, Ports.rightRearTurningMotorPort,
              Ports.rightRearCanCoderPort) };
      m_drive = new Drive(new GyroIOPigeon(Constants.Ports.pigeonPort), Constants.DriveConstants.startPose,
          m_swerveModuleIOs);
      m_throughboreSparkMaxIntakeMotor = new CANSparkMax(Constants.Ports.intakeMotorPort, MotorType.kBrushless);

      m_throughboreSparkMaxIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
      m_throughboreSparkMaxIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
      m_throughboreSparkMaxIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);

      m_throughboreSparkMaxIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
      m_throughboreSparkMaxIntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
      m_throughboreSparkMaxIntakeMotor.burnFlash();
      m_intake = new Intake(
          new IntakeIONeo550(m_throughboreSparkMaxIntakeMotor, Constants.IntakeConstants.intakeGearRatio),
          Constants.IntakeConstants.intakePIDController);
      m_wrist = new Wrist(new WristIOThroughBoreSparkMaxAlternate(Constants.Ports.wristMotorPort,
          Constants.WristConstants.wristEncoderCPR,
          m_throughboreSparkMaxIntakeMotor.getAbsoluteEncoder(Type.kDutyCycle),
          Units.degreesToRadians(33 + 90)), // 253
          Constants.WristConstants.wristPIDController,
          Constants.WristConstants.wristFeedForward, Constants.WristConstants.kMinAngle,
          Constants.WristConstants.kMaxAngle);
      m_elevator = new Elevator(new ElevatorIONeo(Constants.Ports.elevatorLeaderMotorPort,
          Ports.elevatorFollowerMotorPort, Constants.Ports.elevatorThroughBoreEncoderPortA,
          Ports.elevatorThroughBoreEncoderPortB, Constants.ElevatorConstants.kGearRatio,
          Constants.ElevatorConstants.kEncoderCPR), Constants.ElevatorConstants.elevatorPIDController,
          Constants.ElevatorConstants.elevatorFeedForward, Constants.ElevatorConstants.kMinHeightMeters,
          Constants.ElevatorConstants.kMaxHeightMeters,
          Rotation2d.fromDegrees(90).minus(Constants.ElevatorConstants.kAngle));
      m_cams = new CameraAprilTag[] {
          // new CameraAprilTag(VisionConstants.kfrontCameraName, m_layout, VisionConstants.kfrontCameraTransform,
          //     m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP),
          new CameraAprilTag(VisionConstants.khighCamera, m_layout, VisionConstants.khighCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP),
      };
      m_LED = new LED(Constants.LEDConstants.kLEDPort, Constants.LEDConstants.kLEDLength);

      m_robotState = RobotState.startInstance(m_drive, m_intake, m_elevator, m_wrist);
    } else {
      m_drive = new Drive(new GyroIOPigeon(22), new Pose2d(), new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(), new SwerveModuleIOSim());
      m_elevator = new Elevator(new ElevatorIOSim(), ElevatorConstants.elevatorPIDController,
          ElevatorConstants.elevatorFeedForward, ElevatorConstants.kMinHeightMeters,
          ElevatorConstants.kMaxHeightMeters,
          Rotation2d.fromDegrees(90).minus(Constants.ElevatorConstants.kAngle));
      m_wrist = new Wrist(new WristIOSim(), WristConstants.wristPIDController, WristConstants.wristFeedForward,
          WristConstants.kMinAngle, WristConstants.kMaxAngle);
      m_intake = new Intake(new IntakeIOSim(), IntakeConstants.intakePIDController);
      m_LED = new LED(Constants.LEDConstants.kLEDPort, Constants.LEDConstants.kLEDLength);
      m_robotState = RobotState.startInstance(m_drive, m_intake, m_elevator, m_wrist);
    }

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DriverControls driverControls = new DriverControlsDualFlightStick(
        Constants.OIConstants.kDriverLeftDriveStickPort, Constants.OIConstants.kDriverRightDriveStickPort);
    TeloepDrive teleopDrive = new TeloepDrive(m_drive,
        () -> driverControls.getDriveX(),
        () -> driverControls.getDriveY(),
        () -> driverControls.getDriveZ(),
        Constants.DriveConstants.kDriveDeadband);
    m_drive.setDefaultCommand(teleopDrive);

    OperatorControls operatorControls = new OperatorControlsXbox(5);
    Command intakeConeIn = m_intake.startIntakeAtVoltage(-11, -0);
    Command intakeCubeIn = m_intake.startIntakeAtVoltage(11, 0);
    Command pickUpConeVerticalCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.pickUpConeVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.pickUpConeVerticalCommandSetpoints[1])));

    Command pickUpCubeGroundCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.pickUpCubeGroundCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.pickUpCubeGroundCommandSetpoints[1])),
        intakeCubeIn);

    Command pickUpConeGroundCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.pickUpConeGroundCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.pickUpConeGroundCommandSetpoints[1])),
        intakeConeIn);

    Command intakeFromLoadingStationCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.intakeFromLoadingStationCommand[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.pickUpConeGroundCommandSetpoints[1])),
        m_intake.startIntakeAtVoltage(-11.0, -0.0));
    Command coneMidCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.coneMidCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.coneMidCommandSetpoints[1])));
    Command coneHighCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.coneHighCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.coneHighCommandSetpoints[1])));

    Command cubeMidCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.cubeMidCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.cubeMidCommandSetpoints[1])));
    Command cubeHighCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.cubeHighCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.cubeHighCommandSetpoints[1])));

    Command driveThroughPointsToLoadingStationCommand = new DriveThroughPointsToLoadingStation(m_drive,
        DriveConstants.holonomicDrive,
        () -> driverControls.getDriveX(), () -> driverControls.getDriveY(), () -> driverControls.getDriveZ());
    Command stowCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.stowVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.stowVerticalCommandSetpoints[1])));

    Command dropLoaderStationCommand = Commands.parallel(
        m_elevator.setHeightCommand(SetpointConstants.dropLoadingStationCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(SetpointConstants.dropLoadingStationCommandSetpoints[1])));
    driverControls.goToLoadingStation().whileTrue(driveThroughPointsToLoadingStationCommand);
    driverControls.stowIntakeAndElevator().onTrue(stowCommand);
    operatorControls.setpointMidCone().onTrue(coneMidCommand);
    operatorControls.setpointHighCone().onTrue(coneHighCommand);
    operatorControls.setpointMidCube().onTrue(cubeMidCommand);
    operatorControls.setpointHighCube().onTrue(cubeHighCommand);
    operatorControls.intakeConeTipped().whileTrue(pickUpConeGroundCommand).onFalse(stowCommand);
    operatorControls.intakeConeVertical().whileTrue(pickUpConeVerticalCommand).onFalse(stowCommand);
    operatorControls.intakeCubeGround().whileTrue(pickUpCubeGroundCommand).onFalse(stowCommand);
    operatorControls.intakeFromLoadingStation().onTrue(intakeFromLoadingStationCommand);
    operatorControls.stow().onTrue(stowCommand);
    operatorControls.dropStationButton().onTrue(dropLoaderStationCommand);

    driverControls.resetFieldCentric().onTrue(m_drive.resetCommand());
    driverControls.startIntakeConeInCubeOut().whileTrue(m_intake.startIntakeAtVoltage(11, 0));
    driverControls.startIntakeCubeInConeOut().whileTrue(m_intake.startIntakeAtVoltage(-11, -0));
    // driverControls.setpointMidCone().onTrue(coneMidCommand);
    // driverControls.setpointHighCone().onTrue(coneHighCommand);
    // driverControls.setpointMidCube().onTrue(cubeMidCommand);
    // driverControls.setpointHighCube().onTrue(cubeHighCommand);
    driverControls.intakeTippedCone().onTrue(pickUpConeGroundCommand);
    driverControls.setpointIntakeVerticalCone().onTrue(pickUpConeVerticalCommand);
    // driverControls.setpointIntakeGroundCube().onTrue(pickUpCubeGroundCommand);
    // driverControls.intakeFromLoadingStation().onTrue(intakeFromLoadingStationCommand);
    Command chargeCommand = new ChargeStationBalance(m_drive);
    operatorControls.manualInputOverride().whileTrue(m_wrist.moveCommand(operatorControls::moveWristInput));
    operatorControls.manualInputOverride().whileTrue(m_elevator.moveCommand(operatorControls::moveElevatorInput));
    operatorControls.charge().whileTrue(chargeCommand);
    operatorControls.increasePoseSetpoint().onTrue(Commands.runOnce(() -> {
      m_robotState.increasePoseSetpoint();
    }));
    operatorControls.decreasePoseSetpoint().onTrue(Commands.runOnce(() -> {
      m_robotState.decreasePoseSetpoint();
    }));
    operatorControls.partyButton().whileTrue(m_LED.rainbowCommand());

    Command driveToGridSetpointCommand = new DriveToPoint(m_drive, m_robotState::getPoseSetpoint,
        DriveConstants.holonomicDrive,
        () -> driverControls.getDriveX(), () -> driverControls.getDriveY(), () -> driverControls.getDriveZ());
    driverControls.driveToGridSetpoint().whileTrue(driveToGridSetpointCommand);

  }

  public void onEnabled() {
    configureAllianceSettings();
    m_elevator.setBrakeMode(false);
    m_wrist.setBrakeMode(false);
    m_wrist.reset();
    m_elevator.reset();
  }

  public void onDisabled() {
    m_elevator.setBrakeMode(true);
    m_wrist.setBrakeMode(true);
    m_wrist.reset();
    m_elevator.reset();
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
        return m_autoFactory.getAutoCommand(selectedAuto);
      }
    }
    return m_autoChooser.get();
  }

  public void updateRobotState() {
    if (m_robotState != null) {
      m_robotState.update();
    }

  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathplanner.PathPlannerUtil;
import frc.lib.utils.FieldGeomUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.drive.DriveToNode;
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
    m_autoChooser.addDefaultOption("Do Nothing", Commands.none());

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
      m_drive = new Drive(new GyroIOPigeon(Constants.Ports.pigeonPort), m_swerveModuleIOs);
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
          Constants.WristConstants.wristFeedForward,
          Constants.WristConstants.kMinAngle,
          Constants.WristConstants.kMaxAngle);
      m_elevator = new Elevator(new ElevatorIONeo(Constants.Ports.elevatorLeaderMotorPort,
          Ports.elevatorFollowerMotorPort, Constants.Ports.elevatorThroughBoreEncoderPortA,
          Ports.elevatorThroughBoreEncoderPortB, Constants.ElevatorConstants.kGearRatio,
          Constants.ElevatorConstants.kEncoderCPR), Constants.ElevatorConstants.elevatorPIDController,
          Constants.ElevatorConstants.elevatorFeedForward, Constants.ElevatorConstants.kMinHeightMeters,
          Constants.ElevatorConstants.kMaxHeightMeters,
          Rotation2d.fromDegrees(90).minus(Constants.ElevatorConstants.kAngle));
      m_cams = new CameraAprilTag[] {
          new CameraAprilTag(VisionConstants.klowCameraName, m_layout, VisionConstants.klowCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP),
          new CameraAprilTag(VisionConstants.khighCamera, m_layout, VisionConstants.khighCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP),
      };
      m_LED = new LED(Constants.LEDConstants.kLEDPort, Constants.LEDConstants.kLEDLength);

      m_robotState = RobotState.startInstance(m_drive, m_intake, m_elevator, m_wrist);
    } else {
      m_drive = new Drive(
          new GyroIOPigeon(Ports.pigeonPort),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
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
        driverControls::getDriveForward,
        driverControls::getDriveLeft,
        driverControls::getDriveRotation,
        Constants.DriveConstants.kDriveDeadband);
    m_drive.setDefaultCommand(teleopDrive);

    OperatorControls operatorControls = new OperatorControlsXbox(5);

    Command pickUpConeVerticalCommand = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.kIntakeVerticalCone),
        m_intake.intakeConeCommand());

    Command pickUpCubeGroundCommand = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.kIntakeGroundCube),
        m_intake.intakeCubeCommand());

    Command pickUpConeGroundCommand = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.kIntakeTippedCone),
        m_intake.intakeConeCommand());
    Command pickUpConeGroundCommandDriver = RobotState.getInstance()
        .setpointCommand(Setpoints.kIntakeTippedCone);
    Command pickUpConeVerticalCommandDriver = RobotState.getInstance()
        .setpointCommand(Setpoints.kIntakeVerticalCone);
    Command pickUpCubeGroundCommandDriver = RobotState.getInstance()
        .setpointCommand(Setpoints.kIntakeGroundCube);

    Command intakeFromLoadingStationCommand = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.kIntakeLoadingStation),
        m_intake.intakeConeCommand());

    Command coneMidCommand = RobotState.getInstance().setpointCommand(Setpoints.coneMid);
    Command coneHighCommand = RobotState.getInstance().setpointCommand(Setpoints.coneHigh);

    Command cubeMidCommand = RobotState.getInstance().setpointCommand(Setpoints.cubeMid);
    Command cubeHighCommand = RobotState.getInstance().setpointCommand(Setpoints.cubeHigh);

    Command stowAndHoldCone = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.stow),
        m_intake.holdConeCommand().asProxy());

    Command stowAndHoldCube = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.stow),
        m_intake.holdCubeCommand().asProxy());

    // Command driveThroughPointsToLoadingStationCommand = new DriveThroughPointsToLoadingStation(m_drive,
    //     DriveConstants.holonomicDrive,
    //     () -> driverControls.getDriveX(), () -> driverControls.getDriveY(), () -> driverControls.getDriveZ());

    Command stowCommand = RobotState.getInstance().setpointCommand(Setpoints.stow);

    // Command chargeCommand = Commands.sequence(new ZeroHeading(m_drive),
    //     new ChargeStationBalance(m_drive));
    Command chargeCommand = new ChargeStationBalance(m_drive);

    Command dropLoaderStationCommand = Commands.parallel(
        RobotState.getInstance().setpointCommand(Setpoints.kIntakeDropLoadingStation),
        m_intake.intakeConeCommand());

    // driverControls.goToLoadingStation().whileTrue(driveThroughPointsToLoadingStationCommand);
    driverControls.stowIntakeAndElevator().onTrue(stowCommand);
    operatorControls.setpointMidCone().onTrue(coneMidCommand);
    operatorControls.setpointHighCone().onTrue(coneHighCommand);
    operatorControls.setpointMidCube().onTrue(cubeMidCommand);
    operatorControls.setpointHighCube().onTrue(cubeHighCommand);

    operatorControls.intakeConeTipped()
        .whileTrue(pickUpConeGroundCommand)
        .onFalse(stowAndHoldCone);
    operatorControls.intakeConeVertical()
        .whileTrue(pickUpConeVerticalCommand)
        .onFalse(stowAndHoldCone);
    operatorControls.intakeCubeGround()
        .whileTrue(pickUpCubeGroundCommand)
        .onFalse(stowAndHoldCube);
    operatorControls.intakeFromLoadingStation()
        .whileTrue(intakeFromLoadingStationCommand)
        .onFalse(stowAndHoldCone);

    operatorControls.stow().onTrue(stowCommand);

    operatorControls.dropStationButton()
        .onTrue(dropLoaderStationCommand)
        .onFalse(stowAndHoldCone);

    driverControls.startIntakeConeInCubeOut().whileTrue(m_intake.intakeConeCommand());
    driverControls.startIntakeCubeInConeOut().whileTrue(m_intake.intakeCubeCommand());
    driverControls.resetFieldCentric().onTrue(m_drive.resetPoseAngleCommand());
    // driverControls.setpointMidCone().onTrue(coneMidCommand);
    // driverControls.setpointHighCone().onTrue(coneHighCommand);
    // driverControls.setpointMidCube().onTrue(cubeMidCommand);
    // driverControls.setpointHighCube().onTrue(cubeHighCommand);
    driverControls.zeroElevator().whileTrue(m_elevator.zeroHeightCommand());
    driverControls.toggleLedColor().onTrue(Commands.runOnce(() -> {
      m_LED.toggleColor();
    }));
    // driverControls.setpointIntakeGroundCube().onTrue(pickUpCubeGroundCommand);
    driverControls.intakeTippedCone().onTrue(pickUpConeGroundCommandDriver);
    driverControls.intakeVerticalCone().onTrue(pickUpConeVerticalCommandDriver);
    driverControls.setpointIntakeGroundCube().onTrue(pickUpCubeGroundCommandDriver);
    // driverControls.intakeFromLoadingStation().onTrue(intakeFromLoadingStationCommand);

    operatorControls.manualInputOverride().whileTrue(m_wrist.moveCommand(operatorControls::moveWristInput));
    operatorControls.manualInputOverride().whileTrue(m_elevator.moveCommand(operatorControls::moveElevatorInput));
    operatorControls.charge().whileTrue(chargeCommand);
    operatorControls.increasePoseSetpoint().onTrue(Commands.runOnce(() -> {
      m_robotState.increasePoseSetpoint();
    }));
    operatorControls.decreasePoseSetpoint().onTrue(Commands.runOnce(() -> {
      m_robotState.decreasePoseSetpoint();
    }));
    // operatorControls.partyButton().whileTrue(m_LED.rainbowCommand());

    Command driveToGridSetpointCommand = new DriveToNode(m_drive, new FieldGeomUtil(),
        DriveConstants.holonomicDrive,
        () -> driverControls.getDriveForward(), () -> driverControls.getDriveLeft(),
        () -> driverControls.getDriveRotation());
    driverControls.goToNode().whileTrue(driveToGridSetpointCommand);
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

  public void disabledPeriodic() {
    if (Robot.isSimulation()) {
      return;
    }
    String selectedAuto = m_autoChooser.getSendableChooser().getSelected();
    List<PathPlannerTrajectory> traj = m_autoFactory.loadPathGroupByName(selectedAuto);
    Pose2d desPose = traj.get(0).getInitialPose();
    Pose2d curPose = m_drive.getPose();
    double error = curPose.getTranslation().getDistance(desPose.getTranslation());
    if (error > Units.inchesToMeters(1)) {
      m_LED.setSolidColorNumber(Color.kRed, (int) Math.ceil(Units.metersToInches(error)));
    } else {
      m_LED.setSolidColor(Color.kGreen);
    }

  }

  public void updateRobotState() {
    if (m_robotState != null) {
      m_robotState.update();
    }
  }
}

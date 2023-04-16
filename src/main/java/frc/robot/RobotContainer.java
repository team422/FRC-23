// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import org.littletonrobotics.junction.Logger;
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
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.drive.DriveToCube;
import frc.robot.commands.drive.DriveToNode;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOSim;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOWPI;
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
import frc.robot.util.Pigeon2Accel;

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
  private String m_curSelectedAuto = "none";
  private List<PathPlannerTrajectory> m_traj;
  // private LED m_LED2;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureLogging();
    configureAprilTags();
    configureSubsystems();
    configureButtonBindings();
    configureAuto();
  }

  public void configureLogging() {
    if (!RobotConstants.AScopeLogging) {
      Logger.getInstance().end();
    }
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
    // m_autoChooser.addDefaultOption("three_wall", Commands.none());

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
      GyroIOPigeon pigeon = new GyroIOPigeon(Constants.Ports.pigeonPort, Constants.DriveConstants.pitchAngle);
      m_drive = new Drive(pigeon,
          new AccelerometerIOWPI(new Pigeon2Accel(Constants.Ports.pigeonPort)),
          Constants.DriveConstants.startPose,
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
          Units.degreesToRadians(334)), // 118 is back of wrist
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
          new CameraAprilTag(VisionConstants.kleftCameraName, m_layout, VisionConstants.kleftCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP, VisionConstants.kAprilTagPipelineIndex,
              VisionConstants.ksideCameraVFOV, VisionConstants.ksideCameraHFOV),
          new CameraAprilTag(VisionConstants.kRightCamera, m_layout, VisionConstants.kRightCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP, VisionConstants.kAprilTagPipelineIndex,
              VisionConstants.ksideCameraVFOV, VisionConstants.ksideCameraHFOV),
          new CameraAprilTag(VisionConstants.kLimelightCameraName, m_layout, VisionConstants.kLimelightCameraTransform,
              m_drive.getPoseEstimator(), PoseStrategy.MULTI_TAG_PNP, VisionConstants.kCubeSearchPipelineIndex,
              VisionConstants.ktopCameraHFOV, VisionConstants.ktopCameraVFOV)
      };

      m_LED = new LED(Constants.LEDConstants.kLEDPort, Constants.LEDConstants.kLEDLength);
      // m_LED2 = new LED(Constants.LEDConstants.kLEDPort2, Constants.LEDConstants.kLEDLength);

      m_robotState = RobotState.startInstance(m_drive, m_intake, m_elevator, m_wrist);
    } else {
      m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()),
          new AccelerometerIOSim(),
          new Pose2d(),
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
      // m_LED2 = new LED(Constants.LEDConstants.kLEDPort2, Constants.LEDConstants.kLEDLength);
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
        Constants.OIConstants.kDriverLeftDriveStickPort, Constants.OIConstants.kDriverRightDriveStickPort,
        DriveConstants.kDriveDeadband);
    // TeloepDriveTurnPID teleopDrive = new TeloepDriveTurnPID(m_drive, Constants.DriveConstants.holonomicDrive,
    //     driverControls::getDriveForward,
    //     driverControls::getDriveLeft,
    //     driverControls::getDriveRotation,
    //     Constants.DriveConstants.kDriveDeadband,
    //     DriveConstants.kMaxAcceptedErrorMeters,
    //     DriveConstants.kMaxAcceptedAngleError);
    TeleopDrive teleopDrive = new TeleopDrive(m_drive, driverControls::getDriveForward, driverControls::getDriveLeft,
        driverControls::getDriveRotation, DriveConstants.kDriveDeadband);
    m_drive.setDefaultCommand(teleopDrive);

    OperatorControls operatorControls = new OperatorControlsXbox(5);

    Command pickUpConeVerticalCommand = Commands.parallel(
        RobotState.getInstance().setpointCommandSequential(Setpoints.pickUpConeVerticalCommandSetpoints),
        m_intake.intakeConeCommand());

    Command pickUpCubeGroundCommand = Commands.parallel(
        RobotState.getInstance().setpointCommandParallel(Setpoints.pickUpCubeGroundCommandSetpoints),
        m_intake.intakeCubeCommand());

    Command pickUpConeGroundCommand = Commands.parallel(
        RobotState.getInstance().setpointCommandParallel(Setpoints.pickUpConeGroundCommandSetpoints),
        m_intake.intakeConeCommand());
    Command pickUpConeGroundCommandDriver = RobotState.getInstance()
        .setpointCommandParallel(Setpoints.pickUpConeGroundCommandSetpoints);
    Command pickUpConeVerticalCommandDriver = RobotState.getInstance()
        .setpointCommandParallel(Setpoints.pickUpConeVerticalCommandSetpoints);
    Command pickUpCubeGroundCommandDriver = RobotState.getInstance()
        .setpointCommandParallel(Setpoints.pickUpCubeGroundCommandSetpoints);

    Command lebronJamesConeCommand = Commands.sequence(
        m_elevator.setHeightCommand(Units.inchesToMeters(18)),
        Commands.waitSeconds(0.1),
        m_wrist.testSetAngleCommandOnce(Rotation2d.fromDegrees(20)),
        Commands.waitSeconds(0.1),
        m_intake.dropConeCommand());

    Command coneMidCommand = RobotState.getInstance().setpointCommandSequential(Setpoints.coneMidCommandSetpoints);
    Command coneHighCommand = RobotState.getInstance().setpointCommandSequential(Setpoints.coneHighCommandSetpoints);

    Command cubeMidCommand = RobotState.getInstance().setpointCommandSequential(Setpoints.cubeMidCommandSetpoints);
    Command cubeHighCommand = RobotState.getInstance().setpointCommandSequential(Setpoints.cubeHighCommandSetpoints);

    // Command driveThroughPointsToLoadingStationCommand = new DriveThroughPointsToLoadingStation(m_drive,
    //     DriveConstants.holonomicDrive,
    //     () -> driverControls.getDriveX(), () -> driverControls.getDriveY(), () -> driverControls.getDriveZ());

    Command stowCommand = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.stowVerticalCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.stowVerticalCommandSetpoints[1])));

    // Command chargeCommand = Commands.sequence(new ZeroHeading(m_drive),
    //     new ChargeStationBalance(m_drive));
    Command chargeCommand = new ChargeStationBalance(m_drive);
    operatorControls.charge().whileTrue(chargeCommand);
    operatorControls.setIntakeHighPowerMode().whileTrue(m_intake.setHighPowerMode());
    Command intakeCubeTeleop = Commands.parallel(new DriveToCube(m_drive, () -> {
      return RobotState.getInstance().m_cubePose;
    }, DriveConstants.holonomicDrive, () -> 0.0, () -> 0.0, () -> 0.0),
        RobotState.getInstance().setpointCommandParallel(Setpoints.pickUpCubeGroundCommandSetpoints),
        m_intake.intakeCubeCommand());
    Command dropLoaderStationCommand = Commands.parallel(
        m_elevator.setHeightCommand(Setpoints.dropLoadingStationCommandSetpoints[0]),
        m_wrist.setAngleCommand(Rotation2d.fromDegrees(Setpoints.dropLoadingStationCommandSetpoints[1])));

    // driverControls.goToLoadingStation().whileTrue(driveThroughPointsToLoadingStationCommand);
    // FieldGeomUtil fieldGeomUtil = new FieldGeomUtil();
    driverControls.autoScore().whileTrue(
        RobotState.getInstance().autoScore(() -> {
          return RobotState.getInstance().getScoringPose();
        }, () -> {
          return RobotState.getInstance().getWristPosition();
        }, () -> {
          return Setpoints.distanceToDropCone;
        }));
    driverControls.stowIntakeAndElevator().and(operatorControls.dropStationButton().negate()).onTrue(stowCommand);
    operatorControls.setpointMidCone().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).onTrue(coneMidCommand);
    operatorControls.setpointHighCone().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).onTrue(coneHighCommand);
    operatorControls.setpointMidCube().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).onTrue(cubeMidCommand);
    operatorControls.setpointHighCube().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).onTrue(cubeHighCommand);
    operatorControls.intakeConeTipped().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).whileTrue(pickUpConeGroundCommand).onFalse(stowCommand);
    operatorControls.intakeConeVertical().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).whileTrue(pickUpConeVerticalCommand)
        .onFalse(Commands
            .parallel(RobotState.getInstance().setpointCommandParallel(Setpoints.stowVerticalCommandSetpoints)));
    operatorControls.intakeCubeGround().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).whileTrue(pickUpCubeGroundCommand).onFalse(stowCommand);
    operatorControls.stow().and(operatorControls.heightModifier().negate())
        .and(operatorControls.columnModifier().negate()).onTrue(stowCommand);
    // operatorControls.dropStationButton().and(operatorControls.heightModifier().negate())
    //     .and(operatorControls.columnModifier().negate()).whileTrue(dropLoaderStationCommand).onFalse(stowCommand);
    operatorControls.dropStationButton().whileTrue(dropLoaderStationCommand).onFalse(stowCommand);

    operatorControls.columnModifier().and(operatorControls.firstGrid())
        .onTrue(RobotState.getInstance().setGridCommand(1));
    operatorControls.columnModifier().and(operatorControls.secondGrid())
        .onTrue(RobotState.getInstance().setGridCommand(2));
    operatorControls.columnModifier().and(operatorControls.thirdGrid())
        .onTrue(RobotState.getInstance().setGridCommand(3));
    operatorControls.columnModifier().and(operatorControls.firstColumn())
        .onTrue(RobotState.getInstance().setColumnCommand(1));
    operatorControls.columnModifier().and(operatorControls.secondColumn())
        .onTrue(RobotState.getInstance().setColumnCommand(2));
    operatorControls.columnModifier().and(operatorControls.thirdColumn())
        .onTrue(RobotState.getInstance().setColumnCommand(3));

    // operatorControls.heightModifier().and(operatorControls.low())
    //     .onTrue(RobotState.getInstance().setHeightCommand(1));

    operatorControls.heightModifier().and(operatorControls.mid())
        .onTrue(RobotState.getInstance().setHeightCommand(2));

    operatorControls.heightModifier().and(operatorControls.high())
        .onTrue(RobotState.getInstance().setHeightCommand(3));

    driverControls.resetFieldCentric().onTrue(m_drive.resetCommand(new Pose2d(1.81, 6.9, new Rotation2d())));
    driverControls.startIntakeConeInCubeOut().whileTrue(m_intake.intakeConeCommand());
    driverControls.startIntakeCubeInConeOut().whileTrue(m_intake.intakeCubeCommand());
    driverControls.zeroElevator().onTrue(m_elevator.zeroHeightCommand());
    // driverControls.setpointHighCone().onTrue(coneHighCommand);
    // driverControls.setpointMidCube().onTrue(cubeMidCommand);
    // driverControls.setpointHighCube().onTrue(cubeHighCommand);
    driverControls.intakeTippedCone().onTrue(pickUpConeGroundCommandDriver);
    driverControls.intakeVerticalCone().onTrue(pickUpConeVerticalCommandDriver);
    driverControls.setpointIntakeGroundCube().onTrue(pickUpCubeGroundCommandDriver);
    // driverControls.intakeFromLoadingStation().onTrue(intakeFromLoadingStationCommand);
    driverControls.autoIntakeCube().whileTrue(intakeCubeTeleop);
    operatorControls.manualInputOverride().whileTrue(m_wrist.moveCommand(operatorControls::moveWristInput));
    operatorControls.manualInputOverride().whileTrue(m_elevator.moveCommand(operatorControls::moveElevatorInput));
    operatorControls.charge().whileTrue(chargeCommand);

    driverControls.resetDrive().onTrue(m_drive.resetFirmwareCommand());
    driverControls.lebronJames().onTrue(lebronJamesConeCommand);

    // operatorControls.increasePoseSetpoint().onTrue(Commands.runOnce(() -> {
    //   m_robotState.increasePoseSetpoint();
    // }));
    // operatorControls.decreasePoseSetpoint().onTrue(Commands.runOnce(() -> {
    //   m_robotState.decreasePoseSetpoint();
    // }));
    operatorControls.partyButton().onTrue(m_LED.rainbowCommand());

    Command driveToGridSetpointCommand = new DriveToNode(m_drive, new FieldGeomUtil(),
        DriveConstants.holonomicDrive,
        () -> driverControls.getDriveForward(), () -> driverControls.getDriveLeft(),
        () -> driverControls.getDriveRotation());
    driverControls.goToNode().whileTrue(driveToGridSetpointCommand);
    driverControls.ledCone().onTrue(m_LED.coneCommand());
    driverControls.ledCube().onTrue(m_LED.cubeCommand());

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
    // if (Robot.isSimulation()) {
    //   String selectedAuto = m_autoChooser.getSendableChooser().getSelected();
    //   if (PathPlannerUtil.getExistingPaths().contains(selectedAuto)) {
    //     return m_autoFactory.getAutoCommand(selectedAuto);
    //   }
    // }
    return m_autoChooser.get();
  }

  public void disabledPeriodic() {
    // if (Robot.isSimulation()) {
    //   return;
    // } else {
    // RENABLE
    String selectedAuto = m_autoChooser.getSendableChooser().getSelected();
    if (selectedAuto != m_curSelectedAuto) {
      m_curSelectedAuto = selectedAuto;
      m_traj = m_autoFactory.loadPathGroupByName(selectedAuto);
    }

    if (m_traj != null) {
      Pose2d desPose = m_traj.get(0).transformTrajectoryForAlliance(m_traj.get(0), DriverStation.getAlliance())
          .getInitialHolonomicPose();
      Logger.getInstance().recordOutput("Drive/wantedAutoPose", desPose);

      Pose2d pose = RobotState.getInstance().getCamPositionLowConfidence().toPose2d();

      if (pose != null) {
        // m_LED.setSolidColorNumber(Color.kBlue,
        //     (int) Math.ceil(pose.getTranslation().getDistance(new Translation3d(0, 0, 0))));
        double distanceXY = Units.metersToInches(pose.getTranslation().getDistance(desPose.getTranslation()));
        // System.out.println(distanceXY);
        double distanceTheta = Math.abs(pose.getRotation().getDegrees() - desPose.getRotation().getDegrees());
        if (distanceTheta > 300) {
          distanceTheta = Math.abs(360 - distanceTheta);
        }
        // System.out.println(distanceXY);
        // System.out.println(distanceTheta);
        if (distanceXY > 1) {
          m_LED.setSolidColorNumberCommand(Color.kYellow, Color.kGreen, (int) Math.ceil(distanceXY))
              .ignoringDisable(true).schedule();
        } else if (distanceTheta > 1) {
          m_LED.setSolidColorNumberCommand(Color.kRed, Color.kGreen, (int) Math.ceil(distanceTheta))
              .ignoringDisable(true).schedule();
        } else {
          // System.out.println("kBlue");
          m_LED.setSolidColorCommand(Color.kBlue).ignoringDisable(true).schedule();
        }

      } else {
        m_LED.setSolidColorCommand(Color.kRed);
      }
    } else {
      m_LED.setSolidColorCommand(Color.kBrown);
    }

    // }

    // Pose2d curPose = m_drive.getPose();
    // double error = curPose.getTranslation().getDistance(desPose.getTranslation());
    // if (error > Units.inchesToMeters(1)) {
    //   m_LED.setSolidColorNumber(Color.kRed, (int) Math.ceil(Units.metersToInches(error)));
    // } else {
    //   m_LED.setSolidColor(Color.kGreen);
    // }

  }

  public void updateRobotState() {
    if (m_robotState != null) {
      m_robotState.update();
    }

  }

  public void checkLock() {
    if (Timer.getMatchTime() < 0.125 && DriverStation.isFMSAttached()) {
      // m_drive.removeDefaultCommand();
      m_drive.xBrakeCommand().schedule();
    }
  }
}

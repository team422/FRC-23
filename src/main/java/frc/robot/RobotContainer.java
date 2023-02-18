// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsIO;
import frc.robot.oi.DriverControlsIOFlightStick;
import frc.robot.oi.OperatorControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIONeo;
import frc.robot.subsystems.gyro.GyroIOPigeon;
import frc.robot.subsystems.gyro.GyroSub;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONeo550;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOCamera;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
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
  private GyroSub m_gyro;
  private Intake m_intake;
  private Vision m_limelight;
  private Vision m_frontCamera;
  private Vision m_backCamera;
  private Wrist m_wrist;
  private Elevator m_elevator;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSubsystems();
    configureButtonBindings();
  }

  private void configureSubsystems() {
    if (Robot.isReal()) {
      m_gyro = new GyroSub(new GyroIOPigeon(Constants.Ports.pigeonPort));
      m_drive = new Drive(m_gyro, Constants.DriveConstants.startPose);
      m_intake = new Intake(
          new IntakeIONeo550(Constants.Ports.intakeMotorPort, Constants.IntakeConstants.intakeGearRatio),
          Constants.IntakeConstants.intakePIDController);
      m_limelight = new Vision(new VisionIOLimelight(Constants.VisionConstants.klimelightName,
          Constants.VisionConstants.kAprilTagPipelineIndex, Constants.VisionConstants.kReflectiveTapePipelineIndex),
          Constants.VisionConstants.klimelightTransform, m_drive);
      m_frontCamera = new Vision(new VisionIOCamera(Constants.VisionConstants.kfrontCameraName),
          Constants.VisionConstants.kfrontCameraTransform, m_drive);
      m_backCamera = new Vision(new VisionIOCamera(Constants.VisionConstants.kbackCameraName),
          Constants.VisionConstants.kbackCameraTransform, m_drive);
      m_wrist = new Wrist(new WristIOThroughBoreSparkMaxAlternate(Constants.Ports.wristMotorPort,
          Constants.WristConstants.wristEncoderCPR, 0.0, Constants.WristConstants.wristPIDController));
      m_gyro = new GyroSub(new GyroIOPigeon(Constants.Ports.pigeonPort));
      m_elevator = new Elevator(new ElevatorIONeo(Constants.Ports.elevatorLeaderMotorPort,
          Constants.Ports.elevatorFollowerMotorPort, Constants.Ports.elevatorThroughBoreEncoderPortA,
          Constants.Ports.elevatorThroughBoreEncoderPortB, Constants.ElevatorConstants.elevatorGearRatio,
          Constants.ElevatorConstants.elevatorEncoderCPR));
    } else {
      // m_drive = new Drive();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DriverControlsIO driverControls = new DriverControls(new DriverControlsIOFlightStick(
        Constants.OIConstants.kDriverLeftDriveStickPort, Constants.OIConstants.kDriverRightDriveStickPort));
    OperatorControls operatorControls = new OperatorControls() {
    };

    driverControls.getExampleDriverButton().onTrue(m_drive.brakeCommand());
    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}

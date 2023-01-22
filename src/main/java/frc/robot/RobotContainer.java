// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectricalConstants;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.SingleUserXboxControls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIOWPIWrapper;
import frc.robot.subsystems.drive.gyro.PigeonIO;
import frc.robot.subsystems.drive.module.SwerveModuleIOMK2Neo;
import frc.robot.subsystems.drive.module.SwerveModuleIOSim;

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
      m_drive = new Drive(
          new GyroIOWPIWrapper(new ADXRS450_Gyro()),
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
    } else {
      m_drive = new Drive(
          new PigeonIO(ElectricalConstants.kGyroPort),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim(),
          new SwerveModuleIOSim());
    }
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

    driverControls.testButton().onTrue(ChargeStationBalance.charge(m_drive));
    driverControls.resetPose().onTrue(m_drive.resetPoseCommand(new Pose2d(5, 3, Rotation2d.fromDegrees(0))));

    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
    operatorControls.zeroTurnAbsoluteEncoders().onTrue(DebugCommands.zeroTurnAbsoluteEncoders(m_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public void onDisabled() {
    DebugCommands.brakeAndReset(m_drive).schedule();
  }
}

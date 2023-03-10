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
import frc.robot.oi.OperatorControls;
import frc.robot.oi.XboxOperatorController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;

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
  private LED m_LED;

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
      m_drive = new Drive();
      m_LED = new LED(Constants.ElectricalConstants.kLEDPort, Constants.ElectricalConstants.kLEDLength);
    } else {
      m_drive = new Drive();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //define controllers
    DriverControls driverControls = new DriverControls() {
    };
    OperatorControls operatorControls = new XboxOperatorController(5);

    //define buttons
    driverControls.getExampleDriverButton().onTrue(m_drive.brakeCommand());
    operatorControls.getExampleOperatorButton().onTrue(Commands.print("Operator pressed a button!"));
    operatorControls.partyButton().onTrue(m_LED.togglePartyModeCommand());
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PartyMode;
import frc.robot.oi.MixedXboxJoystickControls;
import frc.robot.oi.UserControls;
import frc.robot.subsystems.led.LED;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final SwerveModule mTest;
  // private final SwerveModule mTest2;
  // SwerveModule m_RightFrontSwerveModule;
  // SwerveModule m_LeftFrontSwerveModule;
  // SwerveModule m_RightRearSwerveModule;
  // SwerveModule m_LeftRearSwerveModule;
  // SwerveModule[] m_SwerveModules;

  // FullSwerveBase m_swerveBase;

  private XboxController myController;

  // WPI_Pigeon2 m_Gyro;
  // Vision m_Vision;
  // SmartDashboard m_SmartDashboard;
  LED m_LED;

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_LED = new LED(Constants.ElectricalConstants.LEDPWMPort, Constants.ElectricalConstants.LEDLength);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // myController = new XboxController(0);
    UserControls controls = new MixedXboxJoystickControls(0, 1, 5);
    controls.getXButtonOperator().onTrue(new PartyMode(m_LED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

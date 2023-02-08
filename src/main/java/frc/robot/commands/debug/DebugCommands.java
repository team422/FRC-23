package frc.robot.commands.debug;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

/**
 * Used to brake the robot then reenable coast mode to move
 */
public class DebugCommands {
  public static CommandBase brakeAndReset(Drive drive) {
    return Commands.sequence(
        // Brake Drivebase
        drive.brakeCommand(),
        drive.setBrakeModeCommand(true),

        // Wait for robot to come to a stop
        Commands.waitSeconds(1.0),

        // Enable coast mode to easily move robot
        drive.setBrakeModeCommand(false)).ignoringDisable(true);
  }

  public static CommandBase putSmartDashboardValue(String key, String value) {
    return Commands.runOnce(() -> SmartDashboard.putString(key, value))
        .ignoringDisable(true);
  }

  public static CommandBase zeroTurnAbsoluteEncoders(Drive drive) {
    return Commands.runOnce(() -> {
      System.out.println("Zeroing turn absolute encoders");
      for (var module : drive.getModules()) {
        module.zeroTurnAbsoluteEncoder();
        module.syncTurnEncoderWithAbsolute();
      }
    }, drive);
  }
}

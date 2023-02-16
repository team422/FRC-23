package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class TeloepDrive extends CommandBase {
  Drive m_drive;

  public TeloepDrive(Drive drive) {
    m_drive = drive;
    addRequirements(drive);

  }

}

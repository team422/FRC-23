package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.util.CustomHolmonomicDrive;

public class CustomAutoGen extends ParallelCommandGroup {
  PIDController pidControllerXY = new PIDController(0.5, 0.0, 0.0);
  CustomHolmonomicDrive holonomicDrive = new CustomHolmonomicDrive(pidControllerXY, pidControllerXY);

  public CustomAutoGen(FullSwerveBase base) {
    addCommands(

        sequence(
            new DriveToPointAuto(base, new Pose2d(1, 1, new Rotation2d()),
                holonomicDrive),
            new DriveToPointAuto(base, new Pose2d(0, 0, new Rotation2d()),
                holonomicDrive)

        ));
  }

}

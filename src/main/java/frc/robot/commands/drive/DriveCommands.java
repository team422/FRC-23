package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.oi.DriverControls;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
  public static BaseDriveCommand dropStationDrive(DriverControls controls, Drive drive) {
    final ProfiledPIDController controller = new ProfiledPIDController(3, 0.02, 0.05,
        new Constraints(Units.degreesToRadians(30), Units.degreesToRadians(20)));
    DriveCommandConfig config = new DriveCommandConfig(
        DriveConstants.kDriveKinematics,
        drive::getPose,
        drive::getModuleStates,
        () -> 0.0,
        drive::drive,
        drive);

    Supplier<Rotation2d> targetAngleSupplier = () -> {
      return DriverStation.getAlliance() == Alliance.Blue
          ? Rotation2d.fromDegrees(90)
          : Rotation2d.fromDegrees(-90);
    };

    Supplier<Double> xInput = () -> controls.getDriveForward() * 0.5;
    Supplier<Double> yInput = () -> controls.getDriveLeft() * 0.5;

    return BaseDriveCommand.builder(config)
        .useFieldRelative(true)
        .usePercentSpeeds(false)
        .withLinearSpeedSuppliers(xInput, yInput)
        .withTargetAngleSupplier(targetAngleSupplier, controller)
        .build();
  }
}

package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.oi.DriverControls;

public class DriveCommands {
  public static BaseDriveCommand fieldRelativeDrive(DriveCommandConfig config, DriverControls controls) {
    return BaseDriveCommand.builder(config)
        .withSpeedConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedRadiansPerSecond)
        .usePercentSpeeds(true)
        .useFieldRelative(true)
        .withSpeedSuppliers(
            controls::driveInputForward,
            controls::driveInputLeft,
            controls::driveInputRotate)
        .build();
  }

  public static BaseDriveCommand robotRelativeDrive(DriveCommandConfig config, DriverControls controls) {
    return BaseDriveCommand.builder(config)
        .withSpeedConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedRadiansPerSecond)
        .usePercentSpeeds(true)
        .useFieldRelative(false)
        .withSpeedSuppliers(
            controls::driveInputForward,
            controls::driveInputLeft,
            controls::driveInputRotate)
        .build();
  }

  public static BaseDriveCommand joystickAngleDrive(DriveCommandConfig config, DriverControls controls) {
    Constraints constraints = new Constraints(DriveConstants.kMaxSpeedRadiansPerSecond, Units.degreesToRadians(5));
    ProfiledPIDController controller = new ProfiledPIDController(4, 0, 0.03, constraints);
    return BaseDriveCommand.builder(config)
        .withSpeedConstraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedRadiansPerSecond)
        .useFieldRelative(true)
        .usePercentSpeeds(true)
        .withSafeTargetAngleSupplier(() -> {
          double x = controls.driveInputJoystickAngleX();
          double y = controls.driveInputJoystickAngleY();

          if (Math.hypot(x, y) < 0.7) {
            return Optional.empty();
          }

          return Optional.of(new Rotation2d(x, y));
        }, controller)
        .withLinearSpeedSuppliers(
            controls::driveInputForward,
            controls::driveInputLeft)
        .build();
  }
}

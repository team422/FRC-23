// package frc.robot.constants;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.util.Units;

// public final class DriveConstants {
//   public static final double kRobotLengthMeters = Units.inchesToMeters(36);

//   public static final double kWheelDiameter = Units.inchesToMeters(3.0);
//   public static final double kWheelBase = Units.inchesToMeters(23);
//   public static final double kTrackWidth = Units.inchesToMeters(23);

//   public static final Translation2d[] kSwerveModuleTranslations = {
//       new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0), //values for front left (+, +)
//       new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0), //values for front right (+, -)
//       new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0), //values for back left (-, +)
//       new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0) //values for back right (-, -)
//   };

//   public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kSwerveModuleTranslations);
//   public DriveConstants(double kRobotLengthInches, double kWheelDiameterInches, double kWheelBaseInches, double kTrackWidthInches){
//     kRobotLengthMeters = Units.inchesToMeters(kRobotLengthInches);
//     kWheelDiameter = Units.inchesToMeters(kWheelDiameterInches);
//     kWheelBase = Units.inchesToMeters(kWheelBaseInches);
//     kTrackWidth = Units.inchesToMeters(kTrackWidthInches);

//   }
// }

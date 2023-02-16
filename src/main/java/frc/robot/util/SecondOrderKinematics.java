package frc.robot.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModuleAcceleration;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  WPI_Pigeon2 m_Gyro = new WPI_Pigeon2(DriveConstants.kGyroPort);
  Pigeon2Accelerometer m_Accelerometer = new Pigeon2Accelerometer(m_Gyro);

  /**
  * Get a module's X Acceleration component
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelXMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations,
      SwerveModuleState[] moduleStates, Rotation2d[] modulethetaVel, double robotVel, Rotation2d robotThetaVel) {
    SwerveModuleAcceleration[] moduleAccelsX = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesThetaVelMRobot = new Rotation2d[4];
    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesThetaMRobot[i] = moduleStates[i].angle.minus(Rotation2d.fromDegrees(m_Gyro.getCompassHeading()));
      modulesThetaVelMRobot[i] = modulethetaVel[i].minus(robotThetaVel);
    }

    for (int i = 0; i < 4; i++) {
      moduleAccelsX[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesThetaMRobot[i].getCos())
              - robotVel * modulesThetaVelMRobot[i].getDegrees() * modulesThetaMRobot[i].getSin());
    }
    //Formulae used:
    // thetaMRobot = thetaM - thetaRobot
    // thetaVelMRobot = thetaVelM = thetaVelRobot
    // A_mx = getModuleAccelMatersPerSecondSquared() * Math.cos(thetaMRobot) - botVel * thetaVelMRobot * Math.sin(thetaMRobot);
    return moduleAccelsX;
  }

  /**
  * Get a module's Y Acceleration component
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelYMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations,
      SwerveModuleState[] moduleStates, Rotation2d[] modulethetaVel, double robotVel, Rotation2d robotThetaVel) {
    SwerveModuleAcceleration[] moduleAccelsY = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesThetaVelMRobot = new Rotation2d[4];
    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesThetaMRobot[i] = moduleStates[i].angle.minus(Rotation2d.fromDegrees(m_Gyro.getCompassHeading()));
      modulesThetaVelMRobot[i] = modulethetaVel[i].minus(robotThetaVel);
    }

    for (int i = 0; i < 4; i++) {
      moduleAccelsY[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesThetaMRobot[i].getSin())
              + robotVel * modulesThetaVelMRobot[i].getDegrees() * modulesThetaMRobot[i].getCos());
    }
    //Formulae used:
    // thetaMRobot = thetaM - thetaRobot
    // thetaVelMRobot = thetaVelM = thetaVelRobot
    // A_my = getModuleAccelMatersPerSecondSquared() * Math.sin(thetaMRobot) + botVel * thetaVelMRobot * Math.cos(thetaMRobot);
    return moduleAccelsY;
  }

}

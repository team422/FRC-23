package frc.robot.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  SwerveModuleState[] oldModuleStates = new SwerveModuleState[4];
  SwerveModuleAcceleration[] oldModuleAccelerationsX = new SwerveModuleAcceleration[4];
  SwerveModuleAcceleration[] oldModuleAccelerationsY = new SwerveModuleAcceleration[4];
  Rotation2d[] oldModuleTheta = new Rotation2d[4];

  WPI_Pigeon2 m_Gyro = new WPI_Pigeon2(DriveConstants.kGyroPort);
  Pigeon2Accelerometer m_Accelerometer = new Pigeon2Accelerometer(m_Gyro);

  /**
  * Get a module's X Acceleration component
  * @param moduleAccelerations Accelerations of SwerveBase modules
  * @param moduleStates SwerveModuleStates of the SwerveBase
  * @param moduleThetaVel Theta Velocity of each module
  * @param moduleVelocity Velocity of each module's drive motor
  * @param robotThetaVel Theta Velocity of the robot
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelXMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations, //done
      SwerveModuleState[] moduleStates, //done
      Rotation2d[] moduleThetaVel, //done
      double[] moduleVelocity, //will be done
      Rotation2d robotThetaVel) { //done
    SwerveModuleAcceleration[] moduleAccelsX = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesThetaVelMRobot = new Rotation2d[4];

    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesThetaMRobot[i] = moduleStates[i].angle.minus(Rotation2d.fromDegrees(m_Gyro.getCompassHeading()));
      modulesThetaVelMRobot[i] = moduleThetaVel[i].minus(robotThetaVel);
    }

    //Calcuate Module X Accels
    for (int i = 0; i < 4; i++) {
      moduleAccelsX[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesThetaMRobot[i].getCos())
              - moduleVelocity[i] * modulesThetaVelMRobot[i].getDegrees() * modulesThetaMRobot[i].getSin());
    }

    //Formulae used:
    // thetaMRobot = thetaM - thetaRobot
    // thetaVelMRobot = thetaVelM = thetaVelRobot
    // A_mx = getModuleAccelMatersPerSecondSquared() * Math.cos(thetaMRobot) - moduleVelocity * thetaVelMRobot * Math.sin(thetaMRobot);
    return moduleAccelsX;
  }

  /**
  * Get a module's Y Acceleration component
  * @param moduleAccelerations Accelerations of SwerveBase modules
  * @param moduleStates SwerveModuleStates of the SwerveBase
  * @param moduleThetaVel Theta Velocity of each module
  * @param moduleVelocity Velocity of each module's drive motor
  * @param robotThetaVel Theta Velocity of the robot
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelYMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations, //done
      SwerveModuleState[] moduleStates, //done
      Rotation2d[] moduleThetaVel, //done
      double[] moduleVelocity, //will be done
      Rotation2d robotThetaVel) { //done
    SwerveModuleAcceleration[] moduleAccelsY = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesThetaVelMRobot = new Rotation2d[4];

    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesThetaMRobot[i] = moduleStates[i].angle.minus(Rotation2d.fromDegrees(m_Gyro.getCompassHeading()));
      modulesThetaVelMRobot[i] = moduleThetaVel[i].minus(robotThetaVel);
    }

    //Calculate Module Y Accels
    for (int i = 0; i < 4; i++) {
      moduleAccelsY[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesThetaMRobot[i].getSin())
              + moduleVelocity[i] * modulesThetaVelMRobot[i].getDegrees() * modulesThetaMRobot[i].getCos());
    }

    //Formulae used:
    // thetaMRobot = thetaM - thetaRobot
    // thetaVelMRobot = thetaVelM = thetaVelRobot
    // A_my = getModuleAccelMatersPerSecondSquared() * Math.sin(thetaMRobot) + moduleVelocity * thetaVelMRobot * Math.cos(thetaMRobot);
    return moduleAccelsY;
  }

  //calc integral of accelXY and thetaVel

  public SwerveModuleState[] getVelXYFromAccelXY(SwerveModuleAcceleration[] moduleAccelerations,
      SwerveModuleState[] moduleStates,
      Rotation2d[] moduleThetaVel,
      double[] moduleVelocity,
      Rotation2d robotThetaVel) {

    //init values
    SwerveModuleState[] velXYfromAccelXY = new SwerveModuleState[4];
    SwerveModuleAcceleration[] accelX = calculateModulesPositionAccelXMetersPerSecondSquared(moduleAccelerations,
        moduleStates, moduleThetaVel, moduleVelocity, robotThetaVel);
    SwerveModuleAcceleration[] accelY = calculateModulesPositionAccelYMetersPerSecondSquared(moduleAccelerations,
        moduleStates, moduleThetaVel, moduleVelocity, robotThetaVel);
    double[] velXFromAccel = new double[4];
    double[] velYFromAccel = new double[4];
    Rotation2d[] thetaFromAccel = new Rotation2d[4];
    //Create and set velX, velY, and theta from moduleaccels and modulethetavels
    for (int i = 0; i < 4; i++) {
      velXFromAccel[i] = (accelX[i].getAccel() - oldModuleAccelerationsX[i].getAccel()) * 0.01 //dt times 1/2 to get average and then multiply by height
          + oldModuleStates[i].speedMetersPerSecond;
      velYFromAccel[i] = (accelY[i].getAccel() - oldModuleAccelerationsY[i].getAccel()) * 0.01 //dt times 1/2 to get average and then multiply by height
          + oldModuleStates[i].speedMetersPerSecond;
      thetaFromAccel[i] = Rotation2d
          .fromDegrees((moduleThetaVel[i].getDegrees() - oldModuleStates[i].angle.getDegrees()) * 0.01 //dt times 1/2 to get average and then multiply by height
              + oldModuleStates[i].angle.getDegrees());
    }
    //Create ModuleStates from velx, velY, and Theta
    for (int i = 0; i < 4; i++) {
      double velXY = Math.sqrt(velXFromAccel[i] * velXFromAccel[i] + velYFromAccel[i] * velYFromAccel[i]);
      velXYfromAccelXY[i] = new SwerveModuleState(velXY, thetaFromAccel[i]);
    }
    return velXYfromAccelXY;
  }

  //method to convert to pose2d??
}

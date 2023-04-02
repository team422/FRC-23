package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  SwerveModuleState[] oldModuleStates = new SwerveModuleState[] { new SwerveModuleState(), new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState() };
  SwerveModuleAcceleration[] oldModuleAccelerationsX = new SwerveModuleAcceleration[] {
      new SwerveModuleAcceleration(), new SwerveModuleAcceleration(), new SwerveModuleAcceleration(),
      new SwerveModuleAcceleration() };
  SwerveModuleAcceleration[] oldModuleAccelerationsY = new SwerveModuleAcceleration[] {
      new SwerveModuleAcceleration(), new SwerveModuleAcceleration(), new SwerveModuleAcceleration(),
      new SwerveModuleAcceleration() };
  Rotation2d[] oldModuleTheta = new Rotation2d[] { new Rotation2d(), new Rotation2d(), new Rotation2d(),
      new Rotation2d() };

  /**
  * Get a module's X Acceleration component
  * @param moduleAccelerations Accelerations of SwerveBase modules
  * @param moduleStates SwerveModuleStates of the SwerveBase
  * @param moduleSteerThetaVels Steer Theta Velocity of each module
  * @param moduleVelocities Velocity of each module's drive motor
  * @param robotThetaVel Theta Velocity of the robot
  * @param robotTheta Robot Rotation2d
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelXMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations, //done
      SwerveModuleState[] moduleStates, //done
      Rotation2d[] moduleSteerThetaVels, //done
      double[] moduleVelocities, //done
      Rotation2d robotThetaVel,
      Rotation2d robotTheta) { //done
    SwerveModuleAcceleration[] moduleAccelsX = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesSteerThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesSteerThetaVelMRobot = new Rotation2d[4];

    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesSteerThetaMRobot[i] = moduleStates[i].angle.minus(robotTheta);
      modulesSteerThetaVelMRobot[i] = moduleSteerThetaVels[i].minus(robotThetaVel);
    }

    //Calcuate Module X Accels
    for (int i = 0; i < 4; i++) {
      moduleAccelsX[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesSteerThetaMRobot[i].getCos())
              - moduleVelocities[i] * modulesSteerThetaVelMRobot[i].getDegrees() * modulesSteerThetaMRobot[i].getSin());
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
  * @param moduleSteerThetaVels Steer Theta Velocity of each module
  * @param moduleVelocities Velocity of each module's drive motor
  * @param robotThetaVel Theta Velocity of the robot
  * @param robotTheta Robot Rotation2d
  */
  public SwerveModuleAcceleration[] calculateModulesPositionAccelYMetersPerSecondSquared(
      SwerveModuleAcceleration[] moduleAccelerations, //done
      SwerveModuleState[] moduleStates, //done
      Rotation2d[] moduleSteerThetaVels, //done
      double[] moduleVelocities, //done
      Rotation2d robotThetaVel,
      Rotation2d robotTheta) { //done
    SwerveModuleAcceleration[] moduleAccelsY = new SwerveModuleAcceleration[4];
    Rotation2d[] modulesSteerThetaMRobot = new Rotation2d[4];
    Rotation2d[] modulesSteerThetaVelMRobot = new Rotation2d[4];

    //Convert to robot-centered theta values
    for (int i = 0; i < 4; i++) {
      modulesSteerThetaMRobot[i] = moduleStates[i].angle.minus(robotTheta);
      modulesSteerThetaVelMRobot[i] = moduleSteerThetaVels[i].minus(robotThetaVel);
    }

    //Calculate Module Y Accels
    for (int i = 0; i < 4; i++) {
      moduleAccelsY[i] = new SwerveModuleAcceleration(
          moduleAccelerations[i].accelMetersPerSecondSquared * (modulesSteerThetaMRobot[i].getSin())
              + moduleVelocities[i] * modulesSteerThetaVelMRobot[i].getDegrees() * modulesSteerThetaMRobot[i].getCos());
    }

    //Formulae used:
    // thetaMRobot = thetaM - thetaRobot
    // thetaVelMRobot = thetaVelM = thetaVelRobot
    // A_my = getModuleAccelMatersPerSecondSquared() * Math.sin(thetaMRobot) + moduleVelocity * thetaVelMRobot * Math.cos(thetaMRobot);
    return moduleAccelsY;
  }

  /**
  * Calculates integral of accelXY and thetaVel, returns SwerveModuleState[]
  * @param moduleAccelerations Accelerations of SwerveBase modules
  * @param moduleStates SwerveModuleStates of the SwerveBase
  * @param moduleSteerThetaVels Steer Theta Velocity of each module
  * @param moduleVelocities Velocity of each module's drive motor
  * @param robotThetaVel Theta Velocity of the robot
  * @param robotTheta Robot Rotation2d
  * @param deltaTime Time elapsed since last call, in seconds, ususally one tick, or 0.02s
  */
  public SwerveModuleState[] getModuleStatesFromAccelXY(SwerveModuleAcceleration[] moduleAccelerations,
      SwerveModuleState[] moduleStates,
      Rotation2d[] moduleSteerThetaVels,
      double[] moduleVelocities,
      Rotation2d robotThetaVel,
      Rotation2d robotTheta,
      double deltaTime) {

    //init values
    SwerveModuleState[] modStatesFromAccelXY = new SwerveModuleState[4];

    //calculate AccelX and AccelY
    SwerveModuleAcceleration[] accelX = calculateModulesPositionAccelXMetersPerSecondSquared(moduleAccelerations,
        moduleStates, moduleSteerThetaVels, moduleVelocities, robotThetaVel, robotTheta);
    SwerveModuleAcceleration[] accelY = calculateModulesPositionAccelYMetersPerSecondSquared(moduleAccelerations,
        moduleStates, moduleSteerThetaVels, moduleVelocities, robotThetaVel, robotTheta);

    double[] velXFromAccel = new double[4];
    double[] velYFromAccel = new double[4];
    Rotation2d[] thetaFromAccel = new Rotation2d[4];

    //Create and set velX, velY, and theta from moduleaccels and modulethetavels
    for (int i = 0; i < 4; i++) {
      velXFromAccel[i] = (accelX[i].getAccel() - oldModuleAccelerationsX[i].getAccel()) * deltaTime * 0.5
          + oldModuleStates[i].speedMetersPerSecond;
      velYFromAccel[i] = (accelY[i].getAccel() - oldModuleAccelerationsY[i].getAccel()) * deltaTime * 0.5
          + oldModuleStates[i].speedMetersPerSecond;
      thetaFromAccel[i] = Rotation2d
          .fromDegrees((moduleSteerThetaVels[i].getDegrees() - oldModuleStates[i].angle.getDegrees()) * deltaTime * 0.5
              + oldModuleStates[i].angle.getDegrees());
    }
    //Create ModuleStates from velx, velY, and Theta
    for (int i = 0; i < 4; i++) {
      double velXY = Math.sqrt(velXFromAccel[i] * velXFromAccel[i] + velYFromAccel[i] * velYFromAccel[i]);
      modStatesFromAccelXY[i] = new SwerveModuleState(velXY, thetaFromAccel[i]);
    }
    return modStatesFromAccelXY;
  }

  //method to convert to pose2d??
}

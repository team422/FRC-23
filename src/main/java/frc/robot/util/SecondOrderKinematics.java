package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  /**
  * Get a module's X Acceleration component
  * @param botVel bot's velocity
  * @param botAccel bot's acceleration
  * @param thetaMRobot should be thetaRobot - thetaModule
  * @param thetaVelMRobot should be thetaVelRobot - thetaVelModule
  */
  public static double getModulePositionAccelX(double botVel,
      double botAccel,
      double thetaMRobot,
      double thetaVelMRobot) {
    double Amx = botAccel * Math.cos(thetaMRobot) - botVel * thetaVelMRobot * Math.sin(thetaMRobot);
    return Amx;
  }

  /**
  * Get a module's Y Acceleration component
  * @param botVel bot's velocity
  * @param botAccel bot's acceleration
  * @param thetaMRobot should be thetaRobot - thetaModule
  * @param thetaVelMRobot should be thetaVelRobot - thetaVelModule
  */
  public static double getModulePositionAccelY(double botVel,
      double botAccel,
      double thetaMRobot,
      double thetaVelMRobot) {
    double Amy = botAccel * Math.sin(thetaMRobot) + botVel * thetaVelMRobot * Math.cos(thetaMRobot);
    return Amy;
  }

}

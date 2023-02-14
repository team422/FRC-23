package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  public double getModulePositionAccelX(double botVel,
      double botAccel,
      double thetaMRobot,
      double thetaAccelMRobot) {
    double Amx = botAccel * Math.cos(thetaMRobot) - botVel * thetaAccelMRobot * Math.sin(thetaMRobot);
    return Amx;
  }

  public double getModulePositionAccelY(double botVel,
      double botAccel,
      double thetaMRobot,
      double thetaAccelMRobot) {
    double Amy = botAccel * Math.sin(thetaMRobot) + botVel * thetaAccelMRobot * Math.cos(thetaMRobot);
    return Amy;
  }

}

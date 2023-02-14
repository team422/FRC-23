package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.DriveConstants;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  Pigeon2 m_Gyro = new Pigeon2(DriveConstants.kGyroPort);
  Pigeon2Accelerometer m_Accelerometer = new Pigeon2Accelerometer(m_Gyro);

  double[] accelXYT = m_Accelerometer.getAccelMetersPerSecond();
  double accelX = accelXYT[0];
  double accelY = accelXYT[0];

  private double getBotAccel() {
    return Math.sqrt(accelX * accelX + accelY * accelY);
  }

  private double getThetaMRobot() {
    //Change to get module angle - robot angle later
    return 0;
  }

  private double getThetaVelMRobot() {
    //Change to get module angle vel - robot angle vel later
    return 0;
  }

  /**
  * Get a module's X Acceleration component
  * @param botVel bot's velocity
  * @param thetaMRobot should be thetaModule - thetaRobot
  * @param thetaVelMRobot should be thetaVelModule - thetaVelRobot
  */
  public double getModulePositionAccelX(double botVel,
      double thetaMRobot,
      double thetaVelMRobot) {
    double A_mx = getBotAccel() * Math.cos(thetaMRobot) - botVel * thetaVelMRobot * Math.sin(thetaMRobot);
    return A_mx;
  }

  /**
  * Get a module's Y Acceleration component
  * @param botVel bot's velocity
  * @param thetaMRobot should be thetaModule - thetaRobot
  * @param thetaVelMRobot should be thetaVelModuule - thetaVelRobot
  */
  public double getModulePositionAccelY(double botVel,
      double thetaMRobot,
      double thetaVelMRobot) {
    double A_my = getBotAccel() * Math.sin(thetaMRobot) + botVel * thetaVelMRobot * Math.cos(thetaMRobot);
    return A_my;
  }

}

package frc.robot.util;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.DriveConstants;

public class SecondOrderKinematics extends SwerveDriveKinematics {

  WPI_Pigeon2 m_Gyro = new WPI_Pigeon2(DriveConstants.kGyroPort);
  Pigeon2Accelerometer m_Accelerometer = new Pigeon2Accelerometer(m_Gyro);

  /**
  * Get a module's X Acceleration component
  */
  public double getModulePositionAccelXMetersPerSecondSquared(double botVel,
      double thetaMRobot,
      double thetaVelMRobot) {

    for (int i = 0; i < 0; i++) {

    }
    // double A_mx = getModuleAccelMatersPerSecondSquared() * Math.cos(thetaMRobot)
    //     - botVel * thetaVelMRobot * Math.sin(thetaMRobot);
    return 0;
  }

  /**
  * Get a module's Y Acceleration component
  */
  public double getModulePositionAccelYMetersPerSecondSquared() {

    double[] moduleAccels = new double[4];
    double[] modulePositionAccelY = { 0, 0, 0, 0 };
    for (int i = 0; i < 4; i++) {
      // modulePositionAccelY[i] = FullSwerveBase.getModuleAccels()[i];
      // moduleAccels[i] = FullSwerveBase.getModuleAccels()[i];
    }

    // double A_my = getModuleAccelMatersPerSecondSquared() * Math.sin(thetaMRobot)
    //     + botVel * thetaVelMRobot * Math.cos(thetaMRobot);
    return 0;
  }

}

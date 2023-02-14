package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.Constants.DriveConstants;

public class SecondOrderKinematics extends SwerveDriveKinematics{

  double botVelocity = 0;
  double botAccel = 0;
  double thetaMRobot = 0;
  double thetaAccelMRobot = 0;
  

  public double getModulePositionAccelX(){
    double Amx = Math.cos(thetaMRobot)- Math.sin(thetaMRobot);
    return Amx;
  }

  public double getModulePositionAccelY(){
    double Amy = Math.sin(thetaMRobot) + Math.cos(thetaMRobot);
    return Amy;
  }

}

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.GyroIOPigeon;

public class Balance extends CommandBase {
  Drive m_SwerveBase;
  GyroIOPigeon m_Gyro;
  PIDController m_PIDController;
  double m_kP;
  double m_kI;
  double m_kD;
  double m_PIDScalar;

  public void balance(Drive SwerveBase, GyroIOPigeon Gyro,
      double kP, double kD, double kI, double PIDScalar) {
    m_SwerveBase = SwerveBase;
    m_Gyro = Gyro;
    this.m_kP = kP;
    this.m_kI = kI;
    this.m_kD = kD;
    this.m_PIDScalar = PIDScalar;
    m_PIDController = new PIDController(m_kP, m_kI, m_kD);
    m_PIDController.setTolerance(0.5, 0.1);
  }

  private Translation2d getUp() {
    Translation2d upVec = new Translation2d();
    return upVec;
  }

  @Override
  public void execute() {
    //TODO: multipy to make reasonable (speed *= PIDScalar)
    double speed = m_PIDController.calculate(getUp().getAngle().getDegrees(), 90.0);
    speed *= m_PIDScalar;
    m_SwerveBase.drive(new ChassisSpeeds(speed, 0, 0));
  }

  @Override
  public boolean isFinished() {
    m_SwerveBase.brake();
    return false;
  }
}

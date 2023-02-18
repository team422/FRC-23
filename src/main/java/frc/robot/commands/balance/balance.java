package frc.robot.commands.balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.gyro.GyroSub;

/*
if the current balance is within +/- 2.5 degrees from the desired balance
  w easy
else
  if current balance > desired balance
    move backwards
  if current balance < desired balance
    move forwards

also make sure you're aligned with the platform by turning


 */
public class balance extends CommandBase {
  private final GyroSub m_Gyro;
  private final Drive m_Drive;
  private final PIDController m_Controller;

  public balance(PIDController controller, Drive drive, GyroSub gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    // use subsytem gyro
    m_Gyro = gyro;
    m_Drive = drive;
    m_Controller = controller;
    m_Controller.setTolerance(3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Controller.atSetpoint()) {
      double currDegrees = m_Gyro.getPitch().getDegrees();
      double currVelocity = m_Controller.calculate(currDegrees, 90.0);
      // if (currDegrees > 90) {
      //   currVelocity *= -1;
      // }
      m_Drive.drive(new ChassisSpeeds(currVelocity, 0.0, 0.0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

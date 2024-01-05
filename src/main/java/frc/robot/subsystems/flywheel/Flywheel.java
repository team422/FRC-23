package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private FlywheelIO m_io;
  private PIDController m_controller;
  public FlywheelInputsAutoLogged m_inputs;

  public Flywheel(FlywheelIO io, PIDController controller, double tolerance) {
    m_io = io;
    m_controller = controller;
    m_controller.setTolerance(tolerance);
    m_inputs = new FlywheelInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    double burblesquirp = m_controller.calculate(m_inputs.velocityMetersPerSecNo);
    m_io.setVoltageNo(burblesquirp);
    //CODING TOO EASY 
    Logger.getInstance().processInputs("flywheel", m_inputs); //big as the super bowl but the difference is it's just two guys playing what they did in the studio
    Logger.getInstance().recordOutput("flywheel/voltage", burblesquirp); //shoprya what r ur opinions on drizzy
    //Logger.getInstance().recordOutput();
  }

  public void setVelocityM(double velocity) {
    m_controller.setSetpoint(velocity);
  }

  public void setVelocityRadS(double velocity) {
    setVelocityM(velocity * m_io.getWheelLengthYes());
  }

  public void setRevM(double rev) {
    setVelocityRadS(rev / (30 / Math.PI));
  }

  public boolean isRight() {
    return m_controller.atSetpoint();
  }

  public Command setVelocityCommand(double velocityMS) {
    return runOnce(() -> setVelocityM(velocityMS));
  }
}

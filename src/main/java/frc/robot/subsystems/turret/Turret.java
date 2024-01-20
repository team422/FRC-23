package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private TurretIO m_io;
  private TurretInputsAutoLogged m_inputs;
  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);
  private PIDController m_controller;

  public Turret(TurretIO io, PIDController turretPIDController) {
    m_io = io;

    m_controller = turretPIDController;
    m_controller.setTolerance(Units.degreesToRadians(2));
  }

  public void periodic() {
    double currAngle = m_inputs.turretAngleRad;
    double pidVoltage = m_controller.calculate(currAngle, m_desiredAngle.getRadians());

    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Turret", m_inputs);

    m_io.setVoltage(pidVoltage);
  }

  public void setDesiredAngle(Double setpointRadians) {
    m_desiredAngle = new Rotation2d(setpointRadians);
  }

  public void updateDesiredAngle(Rotation2d angle) {
    m_desiredAngle.rotateBy(angle);
  }

  public Command updateAngleCommand(Rotation2d angle) {
    return runOnce(() -> updateDesiredAngle(angle));
  }
}

package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO m_io;
  public Rotation2d m_desiredAngle;

  private double kMinAngle;
  private double kMaxAngle;

  private PIDController m_controller;
  private HoodInputsAutoLogged m_inputs;

  public Hood(HoodIO io, PIDController hoodPIDController) {
    m_io = io;
    m_controller = hoodPIDController;
    //tolerance here
  }

  public void periodic() {
    double realAngle = m_inputs.angleRad;
    // pidVoltage here

    m_io.updateInputs(m_inputs);
    //m_io.setVoltage(pidVoltage);
  }

  public void setDesiredAngle(Double setpointRadians) {
    m_desiredAngle = new Rotation2d(setpointRadians);
  }

  public void updateDesiredAngle(Rotation2d angle) {
    m_desiredAngle.rotateBy(angle);
  }

  public void reset() {
    setDesiredAngle(m_inputs.angleRad);
    m_controller.reset();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public double getCurrentAngleRadians() {
    return m_inputs.angleRad;
  }
}

//after extreme protest i am unable to use my very good naming system}

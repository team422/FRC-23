package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final SwerveModuleIO[] m_modules;
  private final SwerveModuleIOInputsAutoLogged[] m_inputs;

  /** Creates a new Drive. */
  public Drive(SwerveModuleIO... modules) {
    m_modules = modules;

    m_inputs = new SwerveModuleIOInputsAutoLogged[modules.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new SwerveModuleIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].updateInputs(m_inputs[i]);
      Logger.getInstance().processInputs("Module" + i, m_inputs[i]);
    }
  }

  public void brake() {
    // Set chassis speeds to 0
  }

  public CommandBase brakeCommand() {
    return runOnce(this::brake);
  }
}

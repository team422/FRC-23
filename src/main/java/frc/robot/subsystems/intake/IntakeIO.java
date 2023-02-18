package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface IntakeIO extends LoggedIO<IntakeIO.IntakeIOInputs> {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeSpeed;
  }

  public void setIntakeVoltage(double voltage);

  public double getIntakeSpeed();

}

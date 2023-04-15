package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface IntakeIO extends LoggedIO<IntakeIO.IntakeInputs> {
  @AutoLog
  public static class IntakeInputs {
    public double intakeSpeed;
    public double intakeOutputCurrent;
  }

  public void setIntakeVoltage(double voltage);

  public double getIntakeSpeed();

  public default void setCurrentLimit(int currentLimit) {
  };

  public boolean hasGamePiece();

}

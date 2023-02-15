package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

public class PartyMode extends CommandBase {
  private final LED led;

  public PartyMode(LED led) {
    this.led = led;
  }

  @Override
  public void initialize() {
    led.togglePartyMode();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}

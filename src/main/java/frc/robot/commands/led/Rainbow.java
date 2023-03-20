package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

public class Rainbow extends CommandBase {
  private final LED m_led;
  private int m_offset;

  public Rainbow(LED led) {
    m_led = led;
  }

  @Override
  public void initialize() {
    m_led.start();
  }

  @Override
  public void execute() {
    int length = m_led.getLength();
    Color[] colors = new Color[length];

    for (int i = 0; i < length; i++) {
      final int hue = (m_offset + (i / length) * 180) % 180;
      colors[i] = Color.fromHSV(hue, 255, 128);
    }

    m_led.setColors(colors);
  }

  @Override
  public void end(boolean interrupted) {
    m_led.stop();
  }
}

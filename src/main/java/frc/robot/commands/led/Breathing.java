package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

public class Breathing extends CommandBase {
  private final LED m_led;
  private final double m_periodScale = 1;
  private final Color m_color;
  private double m_breathPercent;

  public Breathing(LED led, Color color) {
    m_led = led;
    m_color = color;
  }

  @Override
  public void initialize() {
    m_breathPercent = 0;
    m_led.start();
  }

  @Override
  public void execute() {
    m_breathPercent = 0.5 * (Math.sin(Timer.getFPGATimestamp() * m_periodScale) + 1);

    Color color = new Color(
        m_color.red * m_breathPercent,
        m_color.blue * m_breathPercent,
        m_color.green * m_breathPercent);

    m_led.setSolidColor(color);
  }

  @Override
  public void end(boolean interrupted) {
    m_led.stop();
  }
}

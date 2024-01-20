package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

public class Flash extends CommandBase {
  private final LED m_LED;
  private final Color m_color;
  private final int m_orgCounter;
  private int m_counter;
  private boolean m_colorToggle = false;
  private int m_wait;
  private boolean m_started = false;
  //basically lets me have it only initialize once but not run until driver presses the button

  public Flash(LED led, Color color, int counter) {
    m_LED = led;
    m_color = color;
    m_orgCounter = counter;
    m_counter = counter;
  }

  @Override
  public void initialize() {
    m_LED.setSolidColor(Color.kBlack);
  }

  @Override
  public void execute() {
    if (!m_started)
      return;
    m_counter--;
    if (m_wait > 0) {
      m_wait--;
      return;
    } else {
      m_wait = 1;
    }
    if (m_counter > 0) {
      if (m_colorToggle) {
        m_LED.setSolidColor(Color.kBlack);
      } else {
        m_LED.setSolidColor(m_color);
      }
      m_colorToggle = !m_colorToggle;
    } else {
      m_LED.setSolidColor(m_color);
    }
  }

  public void start() {
    m_started = true;
  }

  public void reset() {
    m_started = false;
    m_counter = m_orgCounter;
    m_LED.setSolidColor(Color.kBlack);
  }
}

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

// Overview:
//  This command fades between two colors by calculating the rates to transform the rgb values.
//  It will start on the first color and fade to the second color, wait for 1 second, and then
//  it will then fade back to the first color.

public class Fade extends CommandBase {
  private final LED m_LED;
  private final Color[] m_colors;
  private boolean m_currentDirection;
  // m_currentDirection:
  //  true = color 1 to color 2
  //  false = color 2 to color 1
  private double m_fadePercentage;
  private Color m_currentColor;
  private double[] m_rate;
  private int m_wait;
  // delay between color changes

  public Fade(LED led, Color[] colors) {
    m_LED = led;
    m_colors = colors;

    // random initializations, prevent errors but are never used
    m_currentColor = new Color(0, 0, 0);
    m_rate = new double[3];
  }

  @Override
  public void initialize() {
    m_currentDirection = true;
    m_rate = calcRate(m_colors[0], m_colors[1]);
    m_wait = 0;
    m_currentColor = m_colors[1];
    m_fadePercentage = 0;
  }

  @Override
  public void execute() {
    if (m_wait > 0) {
      m_wait--;
      return;
    }

    if (m_fadePercentage == 100) {
      int c = m_currentDirection ? 1 : 0;
      m_currentColor = m_colors[c];
      m_LED.setSolidColor(m_currentColor);
      m_currentDirection = !m_currentDirection;
      m_wait = 100;
      m_fadePercentage = 0;
      return;
    }

    m_fadePercentage += 0.5;

    double r, g, b;
    r = m_currentColor.red;
    g = m_currentColor.green;
    b = m_currentColor.blue;
    if (m_currentDirection) {
      r += m_rate[0];
      g += m_rate[1];
      b += m_rate[2];
    } else {
      r -= m_rate[0];
      g -= m_rate[1];
      b -= m_rate[2];
    }

    m_currentColor = new Color(r, g, b);

    m_LED.setSolidColor(m_currentColor);
  }

  @Override
  public void end(boolean interrupted) {
  }

  private double[] calcRate(Color color1, Color color2) {
    double[] rate = new double[3];
    rate[0] = (color2.red - color1.red) / 200.0;
    rate[1] = (color2.green - color1.green) / 200.0;
    rate[2] = (color2.blue - color1.blue) / 200.0;
    return rate;
  }

  // FOR DEBUGGING:
  public double getPercentage() {
    return m_fadePercentage;
  }

  public Color getColor() {
    return m_currentColor;
  }
}

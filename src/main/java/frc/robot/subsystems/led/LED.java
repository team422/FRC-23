
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.led.Breathing;
import frc.robot.commands.led.Rainbow;

public class LED extends SubsystemBase {
  private final AddressableLED m_LEDStrip;
  private final AddressableLEDBuffer m_LEDStripBuffer;
  private final Color[] m_colors;
  private boolean m_currentCube = false;
  private boolean m_currentCone = false;

  public LED(int PWMPort, int length) {
    m_LEDStrip = new AddressableLED(PWMPort);
    m_LEDStripBuffer = new AddressableLEDBuffer(length);
    m_LEDStrip.setLength(length);
    m_LEDStrip.start();
    m_colors = new Color[] { new Color(255, 200, 0), new Color(255, 0, 100) };
    setSolidColor(Color.kGreen);
  }

  public void setSolidColor(Color color) {
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      m_LEDStripBuffer.setLED(i, color);
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void setSolidColorNumber(Color color, int length) {
    length = Math.min(length, m_LEDStripBuffer.getLength());
    for (int i = 0; i < length; i++) {
      m_LEDStripBuffer.setLED(i, color);
    }
    if (length < m_LEDStripBuffer.getLength()) {
      for (int i = length; i < m_LEDStripBuffer.getLength(); i++) {
        m_LEDStripBuffer.setLED(i, Color.kGreen);
      }
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void setColors(Color... colors) {
    int length = Math.min(m_LEDStripBuffer.getLength(), colors.length);

    for (int i = 0; i < length; i++) {
      if (colors[i] != null) {
        m_LEDStripBuffer.setLED(i, colors[i]);
      }
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void start() {
    m_LEDStrip.start();
  }

  public void stop() {
    m_LEDStrip.stop();
  }

  public int getLength() {
    return m_LEDStripBuffer.getLength();
  }

  public Color getColor() {
    return m_LEDStripBuffer.getLED(0);
  }

  public Command solidColorCommand(Color color) {
    return runOnce(() -> setSolidColor(color));
  }

  public Command allianceColorCommand() {
    Color color = DriverStation.getAlliance() == Alliance.Blue
        ? Color.kMediumBlue
        : Color.kRed;

    return solidColorCommand(color);
  }

  public Command rainbowCommand() {
    return new Rainbow(this);
  }

  public Command breathingCommand(Color color) {
    return new Breathing(this, color);
  }

  public Command coneCommand() {
    return runOnce(() -> {
      m_currentCone = !m_currentCone;
      if (m_currentCone) {
        m_currentCube = false;
        setSolidColor(m_colors[1]);
      } else {
        setSolidColor(Color.kGreen);
      }
    });
  }

  public Command cubeCommand() {
    return runOnce(() -> {
      m_currentCube = !m_currentCube;
      if (m_currentCube) {
        m_currentCone = false;
        setSolidColor(m_colors[0]);
      } else {
        setSolidColor(Color.kGreen);
      }
    });
  }
}

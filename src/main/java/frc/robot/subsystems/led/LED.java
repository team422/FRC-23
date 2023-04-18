
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.led.Breathing;
import frc.robot.commands.led.Fade;
import frc.robot.commands.led.Rainbow;

public class LED extends SubsystemBase {
  private final AddressableLED m_LEDStrip;
  private final AddressableLEDBuffer m_LEDStripBuffer;
  private final Color[] m_colors;
  private Fade m_fade;

  public LED(int PWMPort, int length) {
    m_LEDStrip = new AddressableLED(PWMPort);
    m_LEDStripBuffer = new AddressableLEDBuffer(length);
    m_LEDStrip.setLength(length);
    m_LEDStrip.start();
    m_colors = new Color[] { new Color(255, 200, 0), new Color(255, 0, 100) };
    setSolidColorCommand(m_colors[0]);

    m_fade = new Fade(this,
        new Color[] { Constants.LEDConstants.kMechTechGreen, Constants.LEDConstants.kChargedUpGold });
  }

  public void setSolidColor(Color color) {
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      m_LEDStripBuffer.setLED(i, color);
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public Command setSolidColorCommand(Color color) {
    return Commands.runOnce(() -> {
      setSolidColor(color);
    });
  }

  public Command setSolidColorNumberCommand(Color color, Color secondColor, int length) {
    return Commands.runOnce(() -> {
      int lengthLeds = Math.min(length, m_LEDStripBuffer.getLength());
      for (int i = 0; i < lengthLeds; i++) {
        m_LEDStripBuffer.setLED(i, color);
      }
      if (lengthLeds < m_LEDStripBuffer.getLength()) {
        for (int i = lengthLeds; i < m_LEDStripBuffer.getLength(); i++) {
          m_LEDStripBuffer.setLED(i, secondColor);
        }
      }
      m_LEDStrip.setData(m_LEDStripBuffer);
    });
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

  public Command allianceColorCommand() {
    Color color = DriverStation.getAlliance() == Alliance.Blue
        ? Color.kMediumBlue
        : Color.kRed;

    return setSolidColorCommand(color);
  }

  public Command rainbowCommand() {
    return new Rainbow(this);
  }

  public Command breathingCommand(Color color) {
    return new Breathing(this, color);
  }

  @Override
  public void periodic() {
    m_fade.execute();
  }

  public void startFade() {
    m_fade.schedule();
  }

  public void stopFade() {
    m_fade.end(true);
  }

}

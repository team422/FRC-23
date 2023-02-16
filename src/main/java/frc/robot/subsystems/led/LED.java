package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private final AddressableLED m_LEDStrip;
  private final AddressableLEDBuffer m_LEDStripBuffer;
  private boolean partyMode;
  private boolean lastOn = false;
  private int m_rainbowFirstPixelHue = 0;

  public LED(int PWMPort, int length) {
    partyMode = false;
    m_LEDStrip = new AddressableLED(PWMPort);
    m_LEDStripBuffer = new AddressableLEDBuffer(length);
    m_LEDStrip.setLength(length);
    m_LEDStrip.start();
  }

  @Override
  public void periodic() {
    //continiously sets the things can be replaced
    if (getPartyMode()) {
      smoothRainbow();
      // cycleRainbow();
      // strobeRainbow();
    } else {
      allianceColor();
    }

  }

  private void allianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      blue();
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      red();
    }
  }

  private void red() {
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LEDStripBuffer.setHSV(i, 0, 255, 128);
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  private void green() {
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LEDStripBuffer.setHSV(i, 240, 255, 128);
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  private void blue() {
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LEDStripBuffer.setHSV(i, 120, 255, 128);
    }
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  private void smoothRainbow() {
    // For every pixel
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDStripBuffer.getLength())) % 180; //larger range
      m_LEDStripBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase to make the rainbow "move"
    // Decreasing this number will make it move slower
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    //set LEDs
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  /**
   * @param startHue the starting hue of the rainbow, default is 0 which is red
   */
  public void smoothRainbow(int startHue) {
    m_rainbowFirstPixelHue = startHue;
    // For every pixel
    for (int i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      // Calculate the hue
      // HSV is a loop so...
      // final var hue = (m_rainbowFirstPixelHue + (i * 300 / m_LEDStripBuffer.getLength())) % 300;
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDStripBuffer.getLength())) % 180;
      // Set the value
      m_LEDStripBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Move
    m_rainbowFirstPixelHue %= 180;
    //set LEDs
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void cycleRainbow() {
    for (var i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      m_LEDStripBuffer.setHSV(i, m_rainbowFirstPixelHue, 255, 128);
    }
    m_rainbowFirstPixelHue += 1;

    m_rainbowFirstPixelHue %= 180;
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void strobeRainbow() { //Warning Resource intensive
    int m_firstPixelHue = 0;
    for (var i = 0; i < m_LEDStripBuffer.getLength(); i++) {
      final var hue = (m_firstPixelHue + (i * 180 / m_LEDStripBuffer.getLength())) % 180;
      if (lastOn) {
        m_LEDStripBuffer.setHSV(i, hue, 0, 0);
      } else {
        m_LEDStripBuffer.setHSV(i, hue, 255, 128);
      }
    }
    lastOn = !lastOn;
    m_LEDStrip.setData(m_LEDStripBuffer);
  }

  public void togglePartyMode() {
    partyMode = !partyMode;
  }

  private boolean getPartyMode() {
    return partyMode;
  }

  public CommandBase togglePartyModeCommand() {
    return runOnce(this::togglePartyMode);
  }

}

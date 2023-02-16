package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private final AddressableLED m_LED_Strip;
  private final AddressableLEDBuffer m_LED_Strip_Buffer;
  private boolean partyMode;
  private boolean lastOn = false;
  private int m_rainbowFirstPixelHue = 0;

  public LED(int PWMPort, int length) {
    partyMode = false;
    m_LED_Strip = new AddressableLED(PWMPort);
    m_LED_Strip_Buffer = new AddressableLEDBuffer(length);
    m_LED_Strip.setLength(length);
    m_LED_Strip.start();
  }

  @Override
  public void periodic() {
    //continiously sets the things can be replaced
    if (getPartyMode()) {
      SmoothRainbow();
      // CycleRainbow();
      // StrobeRainbow();
    } else {
      allianceColor();
    }

  }

  private void allianceColor() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      Blue();
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Red();
    }
  }

  private void Red() {
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LED_Strip_Buffer.setHSV(i, 0, 255, 128);
    }
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  private void Green() {
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LED_Strip_Buffer.setHSV(i, 240, 255, 128);
    }
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  private void Blue() {
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_LED_Strip_Buffer.setHSV(i, 120, 255, 128);
    }
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  private void SmoothRainbow() {
    // For every pixel
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LED_Strip_Buffer.getLength())) % 180; //larger range
      //final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LED_Strip_Buffer.getLength())) % 180; //cleaner loop
      // Set the value
      m_LED_Strip_Buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    //set LEDs
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  /**
   * @param startHue the starting hue of the rainbow, default is 0 which is red
   */
  public void SmoothRainbow(int startHue) {
    m_rainbowFirstPixelHue = startHue;
    // For every pixel
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 300 / m_LED_Strip_Buffer.getLength())) % 300;
      //final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LED_Strip_Buffer.getLength())) % 180;
      // Set the value
      m_LED_Strip_Buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    //set LEDs
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  public void CycleRainbow() {
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      m_LED_Strip_Buffer.setHSV(i, m_rainbowFirstPixelHue, 255, 128);
    }
    m_rainbowFirstPixelHue += 1;

    m_rainbowFirstPixelHue %= 180;
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  public void StrobeRainbow() { //Warning Resource intensive
    int m_firstPixelHue = 0;
    for (var i = 0; i < m_LED_Strip_Buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_firstPixelHue + (i * 180 / m_LED_Strip_Buffer.getLength())) % 180;
      //final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LED_Strip_Buffer.getLength())) % 180;
      // Set the value
      if (lastOn) {
        m_LED_Strip_Buffer.setHSV(i, hue, 0, 0);
      } else {
        m_LED_Strip_Buffer.setHSV(i, hue, 255, 128);
      }
    }
    lastOn = !lastOn;
    m_LED_Strip.setData(m_LED_Strip_Buffer);
  }

  public void togglePartyMode() {
    partyMode = !partyMode;
  }

  private boolean getPartyMode() {
    return partyMode;
  }

}

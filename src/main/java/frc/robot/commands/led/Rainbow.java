package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

public class Rainbow extends CommandBase {
  private final LED m_led;
  private int m_offset;
  private Color[] m_rainbowColors;
  private int m_multiple;
  private int m_wait;

  public Rainbow(LED led) {
    m_led = led;
    // m_rainbowColors = new Color[] { Color.kRed, Color.kOrange, Color.kYellow, LEDConstants.kMechTechGreen, Color.kBlue,
    //     Color.kIndigo, Color.kViolet };
    m_rainbowColors = new Color[] { new Color(255, 100, 100), new Color(55, 175, 252), Color.kWhite,
        new Color(55, 175, 252) };
    // m_rainbowColors = new Color[] { LEDConstants.kMechTechGreen, LEDConstants.kChargedUpGold };
    m_multiple = 5;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (m_wait > 0) {
      m_wait--;
      return;
    }

    m_wait = 10;

    int length = m_led.getLength();
    Color[] colors = new Color[length];
    // System.out.println(m_offset);
    for (int i = 0;; i++) {
      // int i = 0;
      // while (true) {
      for (int j = 0; j < m_multiple; j++) {
        // System.out.println(((i) * m_multiple + j + m_offset / 2));
        // if (((i) * m_multiple + j) >= m_led.getLength()) {
        //   m_led.setColors(colors);
        //   m_offset += 1;
        //   if (m_offset > m_led.getLength()) {
        //     m_offset = 0;
        //   }
        //   return;
        // }
        colors[((i) * m_multiple + j + m_offset) % m_led.getLength()] = m_rainbowColors[(i)
            % m_rainbowColors.length];
        if (((i) * m_multiple + j) > m_led.getLength()) {
          m_led.setColors(colors);
          m_offset += 1;
          // if (m_offset == m_rainbowColors.length) {
          //   m_offset = 0;
          // }
          return;
        }
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    // m_led.setSolidColor(Constants.LEDConstants.kMechTechGreen);
  }
}

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.advantagekit;

import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.lib.listeners.ChangeNotifier;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final boolean kTuningMode = true;
  private static final String kTableKey = "TunableNumbers";

  private final String m_key;
  private final Optional<Double> m_defaultValue;
  private final LoggedDashboardNumber m_dashboardNumber;
  private final ChangeNotifier<Double> m_changeNotifier;

  private LoggedTunableNumber(String dashboardKey, Optional<Double> defaultValue) {
    m_key = kTableKey + "/" + dashboardKey;
    m_defaultValue = defaultValue;

    if (kTuningMode) {
      m_dashboardNumber = new LoggedDashboardNumber(m_key, defaultValue.orElse(0.0));
      m_changeNotifier = ChangeNotifier.of(m_dashboardNumber::get);
    } else {
      m_dashboardNumber = null;
      m_changeNotifier = null;
    }
  }

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this(dashboardKey, Optional.empty());
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey, Optional.of(defaultValue));
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (kTuningMode) {
      return m_dashboardNumber.get();
    }

    return m_defaultValue.orElse(0.0);
  }

  public void addListener(Consumer<Double> listener) {
    m_changeNotifier.addListener(listener);
  }

  public void addListener(Runnable listener) {
    m_changeNotifier.addListener(listener);
  }
}

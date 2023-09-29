package frc.robot.subsystems.elevator;

import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.DataPoint;
import frc.lib.hardwareprofiler.HardwareProfiler;
import frc.lib.hardwareprofiler.PIDAutoTuner;
import frc.lib.hardwareprofiler.PowerConsumptionHelper;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.hardwareprofiler.ProfilingScheduling;
import frc.lib.utils.SubsystemProfiles;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class Elevator extends ProfiledSubsystem {
  public final ElevatorInputsAutoLogged m_inputs;
  private final ElevatorIO m_io;
  // private final ElevatorInputs m_inputs;
  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_elevatorFeedForward;
  private double m_desiredHeight;
  private Rotation2d m_elevatorAngle;
  private double m_elevatorOffsetMeters;
  private double m_maxHeight;

  private double m_lastVelocity;
  private double m_lastTime;

  private boolean m_curZeroing;

  private SubsystemProfiles m_profiles;

  private HashMap<Double, Double> m_lastStallMap;

  public enum ElevatorProfilingSuite {
    kSetpointTime, kSetpointDeltaAtTime, kNone, kResetting, kWaiting
  }

  public enum ElevatorProfiles {
    kDefault, kTuning, kTesting, kZeroing
  }

  // Profiling variables
  public ElevatorProfilingSuite m_currentProfileTest = ElevatorProfilingSuite.kNone;
  public ElevatorProfilingSuite m_lastProfileTest = ElevatorProfilingSuite.kNone;
  public int m_profileTestIndex = 0;
  public HardwareProfiler m_profiler = null;
  public HardwareProfiler m_profiler2 = null;
  public PowerConsumptionHelper testPowerConsumption = null;
  public boolean m_profileTestRunning = false;
  public Double m_testStartTime = null;

  // PID tuning variables
  PIDAutoTuner m_PIDTuner;
  double m_nextTime = 0;

  public Elevator(ElevatorIO io, ProfiledPIDController elevatorPIDController, ElevatorFeedforward elevatorFeedForward,
      double ElevatorOffsetMeters, double maxHeight, Rotation2d elevatorAngle) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();

    m_controller = elevatorPIDController;
    m_controller.setTolerance(Units.inchesToMeters(0.1));
    m_elevatorFeedForward = elevatorFeedForward;

    m_controller = Constants.ElevatorConstants.elevatorPIDController;
    m_controller.setTolerance(Units.inchesToMeters(1.5));
    m_elevatorFeedForward = Constants.ElevatorConstants.elevatorFeedForward;

    m_elevatorAngle = elevatorAngle;
    m_elevatorOffsetMeters = ElevatorOffsetMeters;
    m_maxHeight = maxHeight;
    m_lastVelocity = 0;
    m_lastTime = Timer.getFPGATimestamp();
    m_curZeroing = false;

    HashMap<Enum<?>, Runnable> elevatorPeriodicHash = new HashMap<Enum<?>, Runnable>();
    elevatorPeriodicHash.put(ElevatorProfiles.kDefault, this::defaultPeriodic);
    elevatorPeriodicHash.put(ElevatorProfiles.kTuning, this::tuningPeriodic);
    elevatorPeriodicHash.put(ElevatorProfiles.kTesting, this::testingPeriodic);
    elevatorPeriodicHash.put(ElevatorProfiles.kZeroing, this::zeroingPeriodic);
    Class<? extends Enum<?>> profileEnumClass = ElevatorProfiles.class;
    Enum<?> defaultProfile = ElevatorProfiles.kDefault;
    m_profiles = new SubsystemProfiles(profileEnumClass, elevatorPeriodicHash, defaultProfile);
    m_PIDTuner = new PIDAutoTuner("Elevator", Units.inchesToMeters(30),
        Units.inchesToMeters(0.05));

  }

  public void defaultPeriodic() {
    double dt = Timer.getFPGATimestamp() - m_lastTime;

    double pidVoltage = m_controller.calculate(m_inputs.heightMeters, m_desiredHeight);
    double velocitySetpoint = m_controller.getSetpoint().velocity;
    double accelerationSetpoint = (velocitySetpoint - m_lastVelocity) / dt;
    double feedForwardVoltage = m_elevatorFeedForward.calculate(velocitySetpoint,
        accelerationSetpoint);

    double outputVoltage = pidVoltage + feedForwardVoltage;
    if (Robot.isSimulation()) {
      m_io.setVoltage(pidVoltage);
    } else {
      m_io.setVoltage(pidVoltage);
    }
    // m_io.setVoltage(feedForwardVoltage);
    Logger.getInstance().recordOutput("Elevator/PIDVoltage", pidVoltage);
    Logger.getInstance().recordOutput("Elevator/FFVoltage", feedForwardVoltage);
    // Logger.getInstance().recordOutput("Elevator/OutputVoltage", outputVoltage);

    m_lastVelocity = velocitySetpoint;
    m_lastTime = Timer.getFPGATimestamp();
  }

  public void recordOutputs() {

    Logger.getInstance().recordOutput("Elevator/SetpointInches",
        Units.metersToInches(convertTravelToReal(m_desiredHeight)));
    Logger.getInstance().recordOutput("Elevator/HeightInches", Units.metersToInches(getPositionYMeters()));
    Logger.getInstance().recordOutput("Elevator/XInches", Units.metersToInches(getPositionXMeters()));
  }

  public void tuningPeriodic() {
    m_PIDTuner.saveCurrentValue(m_inputs.heightMeters);
    double kP = m_PIDTuner.getKp();
    double kSetpoint = m_PIDTuner.getSetpoint();
    if (ElevatorConstants.kP.hasChanged() || ElevatorConstants.kI.hasChanged()
        || ElevatorConstants.kD.hasChanged()) {
      m_controller.setP(ElevatorConstants.kP.get());
      m_controller.setI(ElevatorConstants.kI.get());
      m_controller.setD(ElevatorConstants.kD.get());
    }
    if (ElevatorConstants.kKs.hasChanged() || ElevatorConstants.kKv.hasChanged()
        || ElevatorConstants.kKg.hasChanged()) {
      m_elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kKs.get(),
          ElevatorConstants.kKg.get(), ElevatorConstants.kKv.get());
    }
    if (ElevatorConstants.kTuningMode && ElevatorConstants.kManualSetpoint.hasChanged()) {
      setHeight(Units.inchesToMeters(ElevatorConstants.kManualSetpoint.get()));
      System.out.println("The height has changed to " + m_desiredHeight);
    }
    System.out.println(ElevatorConstants.kManualSetpoint.hasChanged());
    // Logger.getInstance().recordOutput("Elevator/kP", kP);
    // if (kP != m_controller.getP() && m_controller.getP() != 1) {
    //   m_controller.setP(1);
    //   m_nextTime = Timer.getFPGATimestamp() + 1.0;
    // }
    // if (Timer.getFPGATimestamp() < m_nextTime) {
    //   setHeight(m_elevatorOffsetMeters + Units.inchesToMeters(5));
    //   defaultPeriodic();
    //   return;
    // } else {
    // m_controller.setP(kP);
    // }

    Logger.getInstance().recordOutput("Elevator/kSetpoint", kSetpoint);

    // setHeight(kSetpoint);
    defaultPeriodic();
  }

  public void enablePIDTuning() {
    setCurrentProfile(ElevatorProfiles.kTuning);
  }

  public void zeroingPeriodic() {
    m_io.setVoltage(-3);
    if (m_inputs.currentAmps > ElevatorConstants.kZeroAmperageThreshold) {
      m_lastStallMap.put(Timer.getFPGATimestamp(), m_inputs.currentAmps);
    }
    if (m_lastStallMap.size() > ElevatorConstants.kZeroAmperageCountThreshold) {
      for (HashMap.Entry<Double, Double> entry : m_lastStallMap.entrySet()) {
        if (Timer.getFPGATimestamp() - entry.getKey() > ElevatorConstants.kZeroAmperageTimeThreshold) {
          m_lastStallMap.remove(entry.getKey());
        }
      }
    }
    m_io.zeroHeight();
    m_profiles.revertToLastProfile();
  }

  public void testingPeriodic() {
    switch (m_currentProfileTest) {
      case kWaiting:
        if (ProfilingScheduling.getInstance().checkReadyNextPoint()) {
          setTestProfile(m_currentProfileTest);
        }
      case kSetpointTime:
        if (m_profiler == null) {
          String name = "Elevator Setpoint Time";
          double time = Timer.getFPGATimestamp();
          int subsystemId = ElevatorConstants.kId;
          int id = 1;
          String[] Units = { "Time", "Height" };
          HardwareProfiler.ProfilingType profilingType = HardwareProfiler.ProfilingType.TIME_TO_SETPOINT;
          int testNumber = 1;
          double[] testParamters = { 1.0 };
          String[] testParameterNames = { "Tolerance Inches" };
          m_profiler = new HardwareProfiler(name, time, id, subsystemId, Units, profilingType, testNumber,
              testParamters, testParameterNames);

          name = "Elevator Power Consumption by Height";
          Units = new String[] { "Power", "Height" };
          id = 3;
          profilingType = HardwareProfiler.ProfilingType.POWER_CONSUMPTION_BY_SETPOINT;
          testNumber = 1;
          testParamters = new double[] { 1.0 };
          testParameterNames = new String[] { "Tolerance Inches" };
          m_profiler2 = new HardwareProfiler(name, time, id, subsystemId, Units, profilingType, testNumber,
              testParamters, testParameterNames);
        }
        double curTestHeight = ElevatorConstants.kElevatorTestHeights[m_profileTestIndex];
        setHeight(curTestHeight);
        if (testPowerConsumption == null) {
          testPowerConsumption = new PowerConsumptionHelper();
        }
        testPowerConsumption.update(m_inputs.currentAmps, m_inputs.setVoltage, Timer.getFPGATimestamp());
        System.out.println(testPowerConsumption.getPowerUsage());
        if (m_testStartTime == null) {
          m_testStartTime = Timer.getFPGATimestamp();
          System.out.println(curTestHeight);
        }
        if (Math.abs(getPositionYMeters() - curTestHeight) < Units.inchesToMeters(1.0)) {
          m_profiler.addDataPoint(new DataPoint(
              new double[] { Timer.getFPGATimestamp() - m_testStartTime, Units.metersToInches(curTestHeight) }));
          m_profiler2.addDataPoint(new DataPoint(
              new double[] { testPowerConsumption.getPowerUsage(), Units.metersToInches(curTestHeight), }));
          m_profileTestIndex++;
          m_testStartTime = null;
          if (m_profileTestIndex >= ElevatorConstants.kElevatorTestHeights.length) {
            m_profileTestIndex = 0;
            m_profiler.toJSON();
            m_profiler2.toJSON();
            m_profiler = null;
            m_profiler2 = null;
            m_testStartTime = null;
            testPowerConsumption = null;
            // setTestProfile(ElevatorProfilingSuite.kSetpointDeltaAtTime);
            ProfilingScheduling.getInstance().setFinishTest(m_currentProfileTest);
          } else {
            setTestProfile(ElevatorProfilingSuite.kResetting);
            testPowerConsumption = null;
          }
        } else {
          defaultPeriodic();
        }
        break;
      case kSetpointDeltaAtTime:
        if (m_profiler == null
            && Units.metersToInches(Math.abs(getCurrentHeightMeters() - m_elevatorOffsetMeters)) > 1) {
          setTestProfile(ElevatorProfilingSuite.kResetting);
          break;
        }
        if (m_profiler == null) {
          String name = "Elevator Setpoint Delta At Time";
          double time = Timer.getFPGATimestamp();
          int subsystemId = ElevatorConstants.kId;
          int id = 2;
          String[] Units = { "Delta Height (inches)", "Expected Height (inches)" };
          HardwareProfiler.ProfilingType profilingType = HardwareProfiler.ProfilingType.DELTA_AT_TIME;
          int testNumber = 1;
          double[] testParamters = { 1.0 };
          String[] testParameterNames = { "Time" };
          m_profiler = new HardwareProfiler(name, time, id, subsystemId, Units, profilingType, testNumber,
              testParamters, testParameterNames);
        }
        double curTestHeightTime = ElevatorConstants.kElevatorTestHeights[m_profileTestIndex];
        setHeight(curTestHeightTime);

        if (m_testStartTime == null) {
          m_testStartTime = Timer.getFPGATimestamp();
          System.out.println(curTestHeightTime);
          System.out.println(Units.metersToInches(Math.abs(getPositionYMeters() - m_elevatorOffsetMeters)));

        }
        if ((Timer.getFPGATimestamp() - m_testStartTime) > 1.0) {
          m_profiler.addDataPoint(new DataPoint(
              new double[] { Units.metersToInches(Math.abs(getPositionYMeters() - curTestHeightTime)),
                  Units.metersToInches(curTestHeightTime) }));
          m_profileTestIndex++;
          m_testStartTime = null;
          if (m_profileTestIndex >= ElevatorConstants.kElevatorTestHeights.length) {
            m_profileTestIndex = 0;
            m_profiler.toJSON();
            m_profiler = null;
            ProfilingScheduling.getInstance().setFinishTest(m_currentProfileTest);
          } else {
            m_testStartTime = null;
            setTestProfile(ElevatorProfilingSuite.kResetting);
          }
        } else {
          defaultPeriodic();
        }
        break;
      case kResetting:
        setHeight(0);
        defaultPeriodic();

        if (Math.abs(getCurrentHeightMeters() - m_elevatorOffsetMeters) < Units.inchesToMeters(1.0)) {
          setTestProfile(m_lastProfileTest);
        }
        break;
    }
  }

  @Override
  public void setTestProfile(Enum<?> profileTest) {
    m_lastProfileTest = m_currentProfileTest;
    m_currentProfileTest = (ElevatorProfilingSuite) profileTest;
  }

  @Override
  public void stopTestProfile() {
    m_currentProfileTest = ElevatorProfilingSuite.kNone;
    m_lastProfileTest = ElevatorProfilingSuite.kNone;
    m_profileTestIndex = 0;
    m_testStartTime = null;
    m_profiler = null;
  }

  public Command runSetpointTimeProfilingCommand() {
    setTestProfile(ElevatorProfilingSuite.kSetpointTime);
    return Commands.waitSeconds(5);
  }

  public Command waitUntilProfileIsDone() {
    return Commands.waitUntil(() -> m_currentProfileTest == ElevatorProfilingSuite.kNone && m_profiles == null);
  }

  public Command runSetpointDeltaAtTimeProfilingCommand() {
    setHeight(0);
    waitUntilAtSetpointCommand();
    setTestProfile(ElevatorProfilingSuite.kSetpointDeltaAtTime);
    return Commands.waitUntil(() -> m_currentProfileTest == ElevatorProfilingSuite.kNone);
  }

  public void periodic() {
    m_profiles.getPeriodicFunction().run();

    m_io.updateInputs(m_inputs);
    recordOutputs();
    Logger.getInstance().processInputs("Elevator", m_inputs);
  }

  public void setCurrentProfile(Enum<?> profile) {
    m_profiles.setCurrentProfile(profile);
  }

  public double getCurrentHeightMeters() {
    return m_inputs.heightMeters;
  }

  public double getDesiredMeters() {
    return convertTravelToReal(m_desiredHeight);
  }

  public double getPositionYMeters() {
    return m_inputs.heightMeters * Math.cos(m_elevatorAngle.getRadians()) + m_elevatorOffsetMeters;
  }

  public double getPositionYMetersSetpoint() {
    return m_desiredHeight * Math.cos(m_elevatorAngle.getRadians()) + m_elevatorOffsetMeters;
  }

  public double getPositionXMeters() {
    return m_inputs.heightMeters * Math.sin(m_elevatorAngle.getRadians());
  }

  public double getPositionXMetersAtHeight(double heightMeters) {
    return heightMeters * Math.sin(m_elevatorAngle.getRadians());
  }

  public double getPositionYMetersAtElevatorX(double xMeters) {
    return (xMeters - m_elevatorOffsetMeters) / Math.tan(m_elevatorAngle.getRadians());
  }

  public double getTravelDistanceMeters() {
    return m_inputs.heightMeters;
  }

  public void setHeight(double heightMeters) {
    heightMeters = MathUtil.clamp(heightMeters, m_elevatorOffsetMeters, m_maxHeight);
    heightMeters -= m_elevatorOffsetMeters;
    heightMeters /= Math.cos(m_elevatorAngle.getRadians());
    m_desiredHeight = heightMeters;
  }

  public void reset() {
    setHeight(convertTravelToReal(m_inputs.heightMeters));
    m_controller.reset(m_inputs.heightMeters);
  }

  public double convertTravelToReal(double heightMeters) {
    heightMeters *= Math.cos(m_elevatorAngle.getRadians());
    heightMeters += m_elevatorOffsetMeters;
    return heightMeters;
  }

  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setHeight(heightMeters));
  }

  public Command setHeightCommandContinously(double heightMeters) {
    return run(() -> setHeight(heightMeters));
  }

  public Command testSetHeightCommand(double heightMeters) {
    return Commands.sequence(
        setHeightCommand(heightMeters),
        Commands.waitSeconds(0.1),
        waitUntilAtSetpointCommand());
  }

  public Command testSetHeightCommand(double heightMeters, double tolerance) {
    return Commands.sequence(
        setHeightCommand(heightMeters),
        Commands.waitSeconds(0.1),
        waitUntilWithinToleranceCommand(tolerance));
  }

  public Command waitUntilWithinToleranceCommand(double tolerance) {
    return Commands.waitUntil(() -> withinTolerance(tolerance));
  }

  public Command waitUntilAtSetpointCommand() {
    return Commands.waitUntil(this::atSetpoint);
  }

  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  public boolean withinTolerance(double tolerance) {
    return Math.abs(m_controller.getGoal().position - m_inputs.heightMeters) < tolerance;
  }

  public Command moveCommand(Supplier<Double> heightDelta) {
    return run(() -> setHeight(m_desiredHeight + heightDelta.get()));
  }

  public Command zeroHeightCommand() {
    return runEnd(() -> {
      m_curZeroing = true;
    }, () -> {
      m_curZeroing = false;
    });
  }

  public void setBrakeMode(boolean mode) {
    m_io.setBrakeMode(mode);
  }
}

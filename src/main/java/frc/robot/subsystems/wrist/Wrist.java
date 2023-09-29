package frc.robot.subsystems.wrist;

import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.hardwareprofiler.PIDAutoTuner;
import frc.lib.hardwareprofiler.ProfiledSubsystem;
import frc.lib.utils.SubsystemProfiles;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

public class Wrist extends ProfiledSubsystem {
  private double kMinAngle;
  private double kMaxAngle;
  private WristIO m_io;
  private WristInputsAutoLogged m_inputs;
  public Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);
  public Rotation2d userWantedAngle = Rotation2d.fromDegrees(0);
  private ProfiledPIDController m_controller;
  private PIDController m_testingController;
  private ArmFeedforward m_feedforward;

  private double m_lastTime;
  private double m_lastVelocitySetpoint;
  private double m_nextTime = 0;

  public enum WristProfilingSuite {
    kSetpointTime, kSetpointDeltaAtTime, kNone, kResetting, kWaiting
  }

  public enum WristProfiles {
    kDefault, kTuning, kTesting, kZeroing
  }

  SubsystemProfiles m_profiles;
  PIDAutoTuner m_PIDTuner;

  public Wrist(WristIO io, ProfiledPIDController wristPIDController, ArmFeedforward feedForward, Rotation2d minAngle,
      Rotation2d maxAngle) {
    m_io = io;
    m_inputs = new WristInputsAutoLogged();

    m_controller = wristPIDController;
    m_feedforward = feedForward;
    m_controller.setTolerance(Units.degreesToRadians(2));

    kMinAngle = minAngle.getRadians();
    kMaxAngle = maxAngle.getRadians();
    m_controller.setIntegratorRange(-0.5, 0.5);
    HashMap<Enum<?>, Runnable> elevatorPeriodicHash = new HashMap<Enum<?>, Runnable>();
    elevatorPeriodicHash.put(WristProfiles.kDefault, this::defaultPeriodic);
    elevatorPeriodicHash.put(WristProfiles.kTuning, this::tuningPeriodic);
    elevatorPeriodicHash.put(WristProfiles.kTesting, this::testingPeriodic);
    elevatorPeriodicHash.put(WristProfiles.kZeroing, this::zeroingPeriodic);
    Class<? extends Enum<?>> profileEnumClass = WristProfiles.class;
    Enum<?> defaultProfile = WristProfiles.kDefault;
    m_profiles = new SubsystemProfiles(profileEnumClass, elevatorPeriodicHash, defaultProfile);
    m_PIDTuner = new PIDAutoTuner("Wrist", Rotation2d.fromDegrees(-45).getRadians(),
        Rotation2d.fromDegrees(0.05).getRadians());
  }

  public void defaultPeriodic() {
    Double curAngle = m_inputs.angleRad;
    double dt = Timer.getFPGATimestamp() - m_lastTime;
    double pidVoltage = m_controller.calculate(curAngle, m_desiredAngle.getRadians());
    if (Rotation2d.fromRadians(m_inputs.angleRad).getCos() < .2) {
      pidVoltage /= 2;
    }
    double positionSetpoint = m_controller.getSetpoint().position;
    // double positionSetpoint = m_controller.
    double velocitySetpoint = m_controller.getSetpoint().velocity;
    double accelerationSetpoint = (velocitySetpoint - m_lastVelocitySetpoint) / dt;
    double feedForwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint,
        accelerationSetpoint);

    double outputVoltage = pidVoltage + feedForwardVoltage;
    // if (m_testingController != null) {
    //   pidVoltage = m_testingController.calculate(curAngle, m_desiredAngle.getRadians());
    //   System.out.println(pidVoltage);
    // }
    if (Robot.isSimulation()) {
      m_io.setVoltage(pidVoltage);
    } else {
      m_io.setVoltage(outputVoltage);

    }
    // m_io.setVoltage(feedForwardVoltage);

    // Logger.getInstance().recordOutput("Wrist/PIDVoltage", pidVoltage);
    // Logger.getInstance().recordOutput("Wrist/FFVoltage", feedForwardVoltage);
    // Logger.getInstance().recordOutput("Wrist/OutputVoltage", outputVoltage);

    m_lastTime = Timer.getFPGATimestamp();
    m_lastVelocitySetpoint = velocitySetpoint;
  }

  public void tuningPeriodic() {
    m_PIDTuner.saveCurrentValue(m_inputs.angleRad);
    double kP = m_PIDTuner.getKp();
    double kSetpoint = m_PIDTuner.getSetpoint();
    m_feedforward = new ArmFeedforward(WristConstants.kWristks.get(), WristConstants.kWristkg.get(),
        WristConstants.kWristkv.get(), WristConstants.kWristka.get());
    m_controller = new ProfiledPIDController(WristConstants.kWristP.get(), WristConstants.kWristI.get(),
        WristConstants.kWristD.get(),
        new Constraints(WristConstants.kWristVelo.get(), WristConstants.kWristAccel.get()));
    // if (kP != m_controller.getP()) {
    //   m_controller = new ProfiledPIDController(kP, 0.0, 0.0,
    //       new Constraints(WristConstants.kWristVelo.get(), WristConstants.kWristAccel.get()));
    //   m_nextTime = Timer.getFPGATimestamp() + .3;
    // }
    // if (Timer.getFPGATimestamp() < m_nextTime) {
    //   setAngle(Rotation2d.fromRadians(kMaxAngle));
    //   defaultPeriodic();
    //   return;
    // }
    setAngle(Rotation2d.fromRadians(kSetpoint));
    System.out.println("kP: " + kP + " kSetpoint: " + kSetpoint);

    Logger.getInstance().recordOutput("Wrist/kPTested", kP);
    Logger.getInstance().recordOutput("Wrist/kSetpointTested", kSetpoint);
    defaultPeriodic();
  }

  public void testingPeriodic() {

  }

  public void zeroingPeriodic() {
  }

  public void enablePIDTuning() {
    m_profiles.setCurrentProfile(WristProfiles.kTuning);
  }

  public void periodic() {
    m_profiles.getPeriodicFunction().run();
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Wrist", m_inputs);
    recordOutputs();
  }

  public void recordOutputs() {
    Logger.getInstance().recordOutput("Wrist/SetpointDegrees", m_desiredAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/AngleDeg", Units.radiansToDegrees(m_inputs.angleRad));
  }

  public void reset() {
    setAngle(Rotation2d.fromRadians(m_inputs.angleRad));
    m_controller.reset(m_inputs.angleRad);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setAngle(Rotation2d angle) {
    m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
    userWantedAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
  }

  public void setAngleToNotBreak(Rotation2d angle) {
    m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_inputs.angleRad);
  }

  public Command setAngleCommand(Rotation2d angle) {
    return run(() -> setAngle(angle)).until(m_controller::atGoal);
  }

  public Command testSetAngleCommandOnce(Rotation2d angle) {
    return runOnce(() -> setAngle(angle));
  }

  public Command testSetAngleCommand(Rotation2d angle) {
    return testSetAngleCommand(angle, Units.degreesToRadians(3.0));
  }

  public Command testSetAngleCommand(Rotation2d angle, double toleranceRadians) {
    return Commands.sequence(
        testSetAngleCommandOnce(angle),
        Commands.waitSeconds(0.1),
        waitUntilWithinTolerance(toleranceRadians));
  }

  public Command waitUntilWithinTolerance(double toleranceRadians) {
    return Commands.waitUntil(() -> withinTolerance(toleranceRadians));
  }

  public boolean withinTolerance(double toleranceRadians) {
    return Math.abs(m_controller.getGoal().position - m_inputs.angleRad) < toleranceRadians;
  }

  public Command moveCommand(Supplier<Double> delta) {
    return run(() -> setAngle(m_desiredAngle.plus(Rotation2d.fromDegrees(delta.get()))));
  }

  public void setBrakeMode(boolean mode) {
    m_io.setBrakeMode(mode);
  }
}

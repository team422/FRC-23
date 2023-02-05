package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static final double kElevatorDrumRadius = Units.inchesToMeters(1.5);
  public static final double kElevatorCarriageMassKg = Units.lbsToKilograms(5);
  public static final double kElevatorGearReduction = 4.4;

  public static final Rotation2d kElevatorAngle = Rotation2d.fromDegrees(55);
  public static final double kElevatorMinHeight = Units.feetToMeters(1.0);
  public static final double kElevatorMaxHeight = Units.feetToMeters(6.0);

  private final ElevatorIO m_io;
  private final ElevatorInputsAutoLogged m_inputs;
  private final MechanismLigament2d m_ligament;

  private double m_setpointMeters;
  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      4.0, 0, 0.4, new Constraints(2.45, 2.45)); // TODO: TUNE
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.1, 1.1, 0.1); // TODO: TUNE

  public Elevator(ElevatorIO io, MechanismLigament2d ligament) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();

    m_ligament = ligament;
    m_ligament.setAngle(kElevatorAngle);
    m_ligament.setColor(new Color8Bit(Color.kBeige));
    m_ligament.setLineWeight(10);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Elevator", m_inputs);

    double outputVoltage = m_feedforward.calculate(m_inputs.speedMetersPerSecond)
        + m_controller.calculate(m_inputs.positionMeters, m_setpointMeters);

    if (m_setpointMeters <= kElevatorMinHeight && m_inputs.lowerLimitSwitch) {
      outputVoltage = 0;
    }

    m_io.setVoltage(outputVoltage);

    m_ligament.setLength(Units.metersToInches(getPositionMeters()));

    Logger.getInstance().recordOutput("Elevator/SetpointMeters", m_setpointMeters);
    Logger.getInstance().recordOutput("Elevator/PositionX", getPositionX());
    Logger.getInstance().recordOutput("Elevator/PositionY", getPositionY());
  }

  public void setPosition(double heightMeters) {
    m_setpointMeters = MathUtil.clamp(heightMeters, kElevatorMinHeight, kElevatorMaxHeight);
  }

  public double getPositionMeters() {
    return m_inputs.positionMeters;
  }

  public double getPositionY() {
    return getPositionMeters() * kElevatorAngle.getSin();
  }

  public double getPositionX() {
    return getPositionMeters() * kElevatorAngle.getCos();
  }

  public boolean isLowerLimitSwitched() {
    return m_inputs.lowerLimitSwitch;
  }

  public void zeroEncoder() {
    m_io.zeroEncoder();
  }

  //#region Commands

  public CommandBase setPositionCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  public CommandBase fullExtendCommand() {
    return runOnce(() -> setPosition(kElevatorMaxHeight));
  }

  public CommandBase fullRetractCommand() {
    return runOnce(() -> setPosition(kElevatorMinHeight));
  }

  public CommandBase zeroCommand() {
    return run(() -> setPosition(0))
        .until(this::isLowerLimitSwitched)
        .andThen(this::zeroEncoder);
  }

  //#endregion
}

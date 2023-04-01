package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class ChargeStationBalance extends CommandBase {
  Drive m_drive;
  PIDController m_rollController;
  PIDController m_turnController;
  double m_xBrakeTime = -1.0;
  double m_oldPitchDegrees;
  double m_dpitch;
  Supplier<Double> m_pitchBasedOnTurnSupplierDeg;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    addRequirements(m_drive);
    m_rollController = new PIDController(.5, 0, 0);
    m_turnController = new PIDController(1, 0, 0);
    m_pitchBasedOnTurnSupplierDeg = () -> {
      return Math.round(m_drive.getPose().getRotation().getSin()) == 1.0 ? m_drive.getGyro().getPitch().getDegrees()
          : -m_drive.getGyro().getPitch().getDegrees();
    };
    m_oldPitchDegrees = m_pitchBasedOnTurnSupplierDeg.get();
  }

  @Override
  public void execute() {
    //tells drive base to drive in the opposite direction of the roll
    //uses field relative so it shouldn't care for if it is forward or backwards
    // if (Math.abs(m_drive.getPose().getRotation().getRadians() - Math.PI) > Units.degreesToRadians(4)) {
    //   m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0,
    //       m_turnController.calculate(m_drive.getPose().getRotation().getRadians(), Math.PI),
    //       m_drive.getPose().getRotation()));
    // } else {
    // Pose3d curPose3d = RobotState.getInstance().getCamPositionLowConfidence();
    // if (curPose3d != null && Math.abs(m_drive.getGyro().getPitch().getRadians()) < Units.degreesToRadians(6)) {
    //   m_drive.drive(DriveConstants.holonomicDrive.calculate(curPose3d.toPose2d(), Setpoints.centerOfChargeStation,
    //       () -> 0.0, () -> 0.0, () -> 0.0, true));
    // } else
    m_dpitch = (m_pitchBasedOnTurnSupplierDeg.get() - m_oldPitchDegrees) / 0.020;
    m_oldPitchDegrees = m_pitchBasedOnTurnSupplierDeg.get();
    Logger.getInstance().recordOutput("Drive/pitchChange", m_dpitch);

    if (m_xBrakeTime < 0.0) {
      if (m_xBrakeTime - Timer.getFPGATimestamp() < 1.0) {
        return;
      }

    } else {
      if (m_dpitch > 1) {
        m_drive.xBrake();
        m_xBrakeTime = Timer.getFPGATimestamp();
      }
    }
    if (Timer.getMatchTime() < .125) {
      m_drive.xBrake();
    } else if (Math.abs(m_pitchBasedOnTurnSupplierDeg.get()) < 2.5) {
      m_drive.xBrake();
    } else if (Math.abs(m_pitchBasedOnTurnSupplierDeg.get()) < 11) {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          m_rollController.calculate(Units.degreesToRadians(m_pitchBasedOnTurnSupplierDeg.get()), 0.0) / 5, 0, 0,
          m_drive.getPose().getRotation()));
    } else {
      m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
          -Units.degreesToRadians(m_pitchBasedOnTurnSupplierDeg.get()) * .7, 0, 0,
          m_drive.getPose().getRotation()));
    }

    // }
    //tells drive base to drive in the opposite direction of the roll if it is forward
    // m_drive.drive(new ChassisSpeeds(m_rollController.calculate(m_drive.getGyro().getRoll().getDegrees(), 0), 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.xBrake();
  }
}

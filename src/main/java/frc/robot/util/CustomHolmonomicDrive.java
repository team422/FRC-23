// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
@SuppressWarnings("MemberName")
public class CustomHolmonomicDrive {
    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseTolerance = new Pose2d(.2, .2, new Rotation2d());
    private double kControlFactorY = 0.2;
    private double kControlFactorX = 0.2;
    // private boolean m_enabled = true;

    private final PIDController m_xController;
    private final PIDController m_yController;

    /**
     * Constructs a holonomic drive controller.
     *
     * @param xController A PID Controller to respond to error in the field-relative x direction.
     * @param yController A PID Controller to respond to error in the field-relative y direction.
     */
    @SuppressWarnings("ParameterName")
    public CustomHolmonomicDrive(PIDController xController, PIDController yController) {
        m_xController = xController;
        m_yController = yController;
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final var eTranslate = m_poseError.getTranslation();
        final var tolTranslate = m_poseTolerance.getTranslation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
                && Math.abs(eTranslate.getY()) < tolTranslate.getY();
    }

    /**
     * Sets the pose error which is considered tolerance for use with atReference().
     *
     * @param tolerance The pose error which is tolerable.
     */
    public void setTolerance(Pose2d tolerance) {
        m_poseTolerance = tolerance;
    }

    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param poseRef The desired pose.
     * @param linearVelocityRefMeters The linear velocity reference.
     * @param angleRef The angular reference.
     * @param angleVelocityRefRadians The angular velocity reference.
     * @return The next output of the holonomic drive controller.
     */
    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(
            Pose2d currentPose,
            Pose2d poseRef,
            Supplier<Double> xSpeed,
            Supplier<Double> ySpeed,
            Supplier<Double> angleRef) {

        // Calculate feedforward velocities (field-relative).
        // double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        // double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        // double thetaFF = angleVelocityRefRadians;
        m_poseError = poseRef.relativeTo(currentPose);

        // Calculate feedback velocities (based on position error).
        double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

        EricNubControls EricControls = new EricNubControls();
        double x_speed = EricControls.addDeadzoneScaled(xSpeed.get(), 0.1);
        double y_speed = EricControls.addDeadzoneScaled(ySpeed.get(), 0.1);

        double x = xFeedback + (kControlFactorX * x_speed);
        double y = yFeedback + (kControlFactorY * y_speed);

        double mag = Math.sqrt((x * x) + (y * y));

        // if you dont want to be at max speed, dont use this function
        double xF = x / mag;
        double yF = y / mag;

        // Calculate feedback velocities (based on angle error).

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(xF * DriveConstants.kMaxSpeedMetersPerSecond,
                yF * DriveConstants.kMaxSpeedMetersPerSecond,
                EricControls.addEricCurve(EricControls.addDeadzoneScaled(angleRef.get(), 0.1))
                        * DriveConstants.kMaxAngularSpeed,
                currentPose.getRotation());
    }
}

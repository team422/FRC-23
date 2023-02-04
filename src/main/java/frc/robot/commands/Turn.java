package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FullSwerveBase;

public class Turn extends CommandBase {
    public FullSwerveBase swerveDrive;
    public double angle;

    public Turn(FullSwerveBase swerveBase, double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveBase);
        swerveDrive = swerveBase;
        this.angle = angle;
    }

    @Override
    public void execute() {
        // swerveDrive.setDesiredTurn(new SwerveModuleState(0.0, new Rotation2d(angle)));
        double turnSpeed = Rotation2d.fromDegrees(angle).getRadians() - swerveDrive.getGyroAngle().getRadians();
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, turnSpeed);
        swerveDrive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(swerveDrive.getGyroAngle().getDegrees() - angle) > 2) {
            return true;
        }
        return false;
    }
}

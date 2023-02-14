// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;

/** An example command that uses an example subsystem. */
public class DriveOneModule extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final SwerveModule swerveModule;
    private final Supplier<Double> xSpeed;
    private final Supplier<Double> ySpeed;
    private final Supplier<Double> zRotation;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveOneModule(SwerveModule swerveModule, Supplier<Double> x, Supplier<Double> y, Supplier<Double> z) {
        this.swerveModule = swerveModule;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.swerveModule);
        xSpeed = x;
        ySpeed = y;
        zRotation = z;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("test" + swerveModule.getState().toString());
        double currentDesiredTurn = zRotation.get() * 180;
        double currentDesiredSpeed = Math.sqrt(Math.pow(xSpeed.get(), 2) + Math.pow(ySpeed.get(), 2));
        // swerveModule.setDesiredState(new SwerveModuleState(0.05, new Rotation2d(90)));
        swerveModule.setDesiredState(new SwerveModuleState(currentDesiredSpeed, new Rotation2d(currentDesiredTurn)));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

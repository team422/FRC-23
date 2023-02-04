package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;

public class GetOffsetValues extends CommandBase {
    public static SwerveModule[] m_SwerveModules;
    public Rotation2d absModuleRotation;

    public GetOffsetValues(SwerveModule[] SwerveModules) {
        this.m_SwerveModules = SwerveModules;
        // addRequirements(drive);
    }

    @Override
    public void execute() {
        for (SwerveModule module : m_SwerveModules) {
            absModuleRotation = module.getAbsoluteRotation();
            System.out.println(absModuleRotation.getDegrees());
            // module.setDesiredState(new SwerveModuleState(0.0, absModuleRotation));
        }
    }

}

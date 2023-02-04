package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveModule;

public class AutoSetSwerveState extends SequentialCommandGroup {
    /**
     * Creates a new AutoTestSequence.
     */
    private final SwerveModule m_swerveModule;

    public AutoSetSwerveState(SwerveModule module, SwerveModuleState state) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super();
        m_swerveModule = module;
        System.out.println("AutoSetSwerveState Desired State:" + state);
        System.out.println("AutoSetSwerveState Current State:" + m_swerveModule.getState());
        System.out.println("AutoSetSwerveState Current State Absolute:" + m_swerveModule.getAbsoluteEncoder());
        m_swerveModule.setDesiredState(state);

        System.out.println("AutoSetSwerveState Desired State:" + state);
        System.out.println("AutoSetSwerveState Current State:" + m_swerveModule.getState());
        System.out.println("AutoSetSwerveState Current State Absolute:" + m_swerveModule.getAbsoluteEncoder());

        // new Turn(m_swerveModule, m_speed),
        // new Turn(m_swerveModule2, m_speed));

    }

}

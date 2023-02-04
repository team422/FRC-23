package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveModule;

public class AutoTestSequence extends SequentialCommandGroup {
    /**
     * Creates a new AutoTestSequence.
     */
    private final SwerveModule m_swerveModule;
    private final SwerveModule m_swerveModule2;
    private final double m_speed;

    public AutoTestSequence(SwerveModule module, SwerveModule module2, double speed) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super();
        m_swerveModule = module;
        m_swerveModule2 = module2;
        m_speed = speed;
        // new Turn(m_swerveModule, m_speed),
        // new Turn(m_swerveModule2, m_speed));

    }

}

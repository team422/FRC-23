// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.SwerveModule;

// public class AutoDriveState extends SequentialCommandGroup {
//     /**
//      * Creates a new AutoTestSequence.
//      */
//     private final FullSwerveDrive m_drive;

//     public AutoDriveState(FullSwerveDrive module, SwerveModuleState state) {
//         // Add your commands in the super() call, e.g.
//         // super(new FooCommand(), new BarCommand());
//         super();
//         m_drive = module;
//         System.out.println("AutoDriveState Desired State:" + state);
//         System.out.println("AutoDriveState Current State:" + m_drive.getState());
//         System.out.println("AutoDriveState Current State:" + m_drive.getAbsoluteEncoder());
//         m_drive.drive(new ChassisSpeeds(2, 0, 0));
//         System.out.println("AutoDriveState Desired State:" + state);
//         System.out.println("AutoDriveState Current State:" + m_drive.getState());
//         System.out.println("AutoDriveState Current State:" + m_drive.getAbsoluteEncoder());

//         // new Turn(m_drive, m_speed),
//         // new Turn(m_drive2, m_speed));

//     }

// }

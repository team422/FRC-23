// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.FullSwerveDrive;
import frc.robot.commands.StartSwerveTestingMode;
import frc.robot.commands.SwitchSwerveWheel;
import frc.robot.oi.MixedXboxJoystickControls;
import frc.robot.oi.UserControls;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;
import frc.robot.util.CustomHolmonomicDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    // private final SwerveModule mTest;
    // private final SwerveModule mTest2;
    SwerveModule m_RightFrontSwerveModule;
    SwerveModule m_LeftFrontSwerveModule;
    SwerveModule m_RightRearSwerveModule;
    SwerveModule m_LeftRearSwerveModule;
    SwerveModule[] m_SwerveModules;

    FullSwerveBase m_swerveBase;

    private XboxController myController;

    WPI_Pigeon2 m_Gyro;
    Vision m_Vision;
    SmartDashboard m_SmartDashboard;

    // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        // mTest = new SwerveModule(Constants.DriveConstants.kSwerveTestMotorDrive,
        //         Constants.DriveConstants.kSwerveTestMotorTurning, Constants.DriveConstants.analogEncoderSwerveTesting,
        //         0);
        // mTest2 = new SwerveModule(Constants.DriveConstants.kSwerveTestMotorDrive2, // SET REAL CONSTANT VALUES
        //         Constants.DriveConstants.kSwerveTestMotorTurning2, Constants.DriveConstants.analogEncoderSwerveTesting2,
        //         1);

        // 0.24643664905961252
        // 0.008663508697619058
        // 0.15306316494355832
        // 0.09232824983462598
        // 0.23578662301985037
        // 0.008170862715626614
        // 0.16252281335580906
        // 0.08742928353572182

        PathPlannerServer.startServer(5811);

        m_RightFrontSwerveModule = new SwerveModule(Constants.DriveConstants.kFrontRightDriveMotor,
                Constants.DriveConstants.kFrontRightTurningMotor, Constants.DriveConstants.kFrontRightEncoder,
                322.8); // 2 266.9
        m_LeftFrontSwerveModule = new SwerveModule(Constants.DriveConstants.kFrontLeftDriveMotor,
                Constants.DriveConstants.kFrontLeftTurningMotor, Constants.DriveConstants.kFrontLeftEncoder,
                324.10); // 3
        m_RightRearSwerveModule = new SwerveModule(Constants.DriveConstants.kRearRightDriveMotor,
                Constants.DriveConstants.kRearRightTurningMotor, Constants.DriveConstants.kRearRightEncoder,
                23.18); // 1
        m_LeftRearSwerveModule = new SwerveModule(Constants.DriveConstants.kRearLeftDriveMotor,
                Constants.DriveConstants.kRearLeftTurningMotor, Constants.DriveConstants.kRearLeftEncoder,
                334.16); // 0 
        m_SwerveModules = new SwerveModule[] { m_LeftFrontSwerveModule, m_RightFrontSwerveModule,
                m_LeftRearSwerveModule,
                m_RightRearSwerveModule };

        m_Gyro = new WPI_Pigeon2(Constants.DriveConstants.kGyroPort);

        // // m_RightFrontSwerveModule = new SwerveModule(Constants.DriveConstants.kFrontRightDriveMotor,
        // //         Constants.DriveConstants.kFrontRightTurningMotor, Constants.DriveConstants.kFrontRightEncoder,
        // //         270);
        // // m_LeftFrontSwerveModule = new SwerveModule(Constants.DriveConstants.kFrontLeftDriveMotor,
        // //         Constants.DriveConstants.kFrontLeftTurningMotor, Constants.DriveConstants.kFrontLeftEncoder, 90);
        // // m_RightRearSwerveModule = new SwerveModule(Constants.DriveConstants.kRearRightDriveMotor,
        // //         Constants.DriveConstants.kRearRightTurningMotor, Constants.DriveConstants.kRearRightEncoder, 90);
        // // m_LeftRearSwerveModule = new SwerveModule(Constants.DriveConstants.kRearLeftDriveMotor,
        // //         Constants.DriveConstants.kRearLeftTurningMotor, Constants.DriveConstants.kRearLeftEncoder, 90);
        // // m_SwerveModules = new SwerveModule[] { m_LeftFrontSwerveModule, m_RightFrontSwerveModule,
        // //         m_LeftRearSwerveModule, m_RightRearSwerveModule };

        // m_Gyro = new ADXRS450_Gyro();

        m_swerveBase = new FullSwerveBase(m_SwerveModules, m_Gyro);

        m_Vision = new Vision(new PhotonCamera(Constants.Vision.firstCameraName), m_swerveBase);

        configureButtonBindings();
    }

    public void printDriveBaseVals() {
        m_swerveBase.printAllVals();
    }

    public void printSingleModuleData() {
        System.out.println("ABSOLUTE" + m_RightRearSwerveModule.getAbsoluteRotation().getDegrees());
        System.out.println("normal" + m_RightRearSwerveModule.getTurnDegrees());

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // myController = new XboxController(0);
        UserControls controls = new MixedXboxJoystickControls(0, 1, 5);

        // new JoystickButton(myController, 1).whenHeld(new DriveOneModule(mTest, () -> myController.getLeftX(),
        //         () -> myController.getLeftY(), () -> myController.getRightX()));
        FullSwerveDrive driveCommand = new FullSwerveDrive(m_swerveBase, () -> -controls.getLeftDriveY(),
                () -> -controls.getLeftDriveX(), () -> -controls.getRightDriveX());// , m_SwerveBase.getHeading()
        m_swerveBase.setDefaultCommand(driveCommand);
        controls.getAButtonOperator().onTrue(new SwitchSwerveWheel(m_swerveBase));
        // while active once is now deprecated
        controls.getBButtonOperator().onTrue(new StartSwerveTestingMode(m_swerveBase));
        controls.resetPoseTrigger().onTrue(
                Commands.runOnce(() -> m_swerveBase.resetPose(new Pose2d(5, 3, new Rotation2d())), m_swerveBase));
        PIDController pidControllerXY = new PIDController(0.05, 0.0, 0.0);
        CustomHolmonomicDrive holonomicDrive = new CustomHolmonomicDrive(pidControllerXY, pidControllerXY);
        DriveToPoint driveToPointTesting = new DriveToPoint(m_swerveBase, new Pose2d(5, 3, new Rotation2d()),
                holonomicDrive, () -> controls.getLeftDriveY(), () -> -controls.getLeftDriveX(),
                () -> -controls.getRightDriveX());
        controls.moveToPoseTrigger().whileTrue(driveToPointTesting);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return m_autoCommand;
        // mTest.setDesiredState(Double);
        // return new AutoTestSequence(mTest2, mTest, 0.2);
        // return new Turn(m_SwerveBase, 50);
        // return new AutoSetSwerveState(m_RightFrontSwerveModule, new SwerveModuleState(0, new Rotation2d(3.14 / 2)));

        var autoPath = PathPlanner.loadPathGroup("Drive testing", new PathConstraints(2, 1));
        List<PathPlannerTrajectory.EventMarker> events;
        Map<String, Command> eventMap = new HashMap<>();
        eventMap.put("brake", m_swerveBase.fullBrakeCommand());
        // return new FollowPath(m_SwerveBase, autoPath);
        PIDConstants linearPIDConstants = new PIDConstants(12, 0, 0);
        PIDConstants angularPIDConstants = new PIDConstants(3, 0, 0);
        BaseAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_swerveBase::getPose,
                m_swerveBase::resetPose,
                linearPIDConstants, angularPIDConstants,
                m_swerveBase::drive,
                eventMap,
                false, m_swerveBase);

        return autoBuilder.fullAuto(autoPath);
    }
}

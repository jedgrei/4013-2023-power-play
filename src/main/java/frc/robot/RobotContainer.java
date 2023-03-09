// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static final SwerveDrive m_swerveDrive = new SwerveDrive();
	// private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController =
		new CommandXboxController(OperatorConstants.kDriverControllerPort);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(m_exampleSubsystem::exampleCondition)
		// 	.onTrue(new ExampleCommand(m_exampleSubsystem));

		// // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// // cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
		0,
		0
		);

		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0, 0, new Rotation2d(0)),
		List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
		new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
		trajectoryConfig
		);

		PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
		PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
		ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
		trajectory,
		m_swerveDrive::getPose,
		DriveConstants.kDriveKinematics,
		xController,
		yController,
		thetaController,
		m_swerveDrive::setModuleStates,
		m_swerveDrive
		);
		return new SequentialCommandGroup(
			new InstantCommand(() -> m_swerveDrive.resetOdometry(trajectory.getInitialPose())),
			swerveControllerCommand,
			new InstantCommand(() -> m_swerveDrive.stopModules())
		);
	}

	// return command to be used periodically in teleop
	public Command getTeleOpCommand() {
		return new SwerveJoystickCommand(
			m_swerveDrive,
			m_driverController::getLeftX,
			m_driverController::getLeftY,
			m_driverController::getRightX,
			(() -> {return false;})
		);
	}
}

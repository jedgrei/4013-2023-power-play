package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ShortDrive;
import frc.robot.commands.SplineTest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Hand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final DriveBase drive = new DriveBase();
	private final Arm arm = new Arm();
	private final Hand hand = new Hand();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController armController = new CommandXboxController(0);
	private final CommandXboxController driveController = new CommandXboxController(1);

	SendableChooser<Command> autoChooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		configureBindings();

		// set up auto chooser
		autoChooser.setDefaultOption("Short Drive", new ShortDrive(drive));
		autoChooser.addOption("Complex Auto", new AutoCommand(drive, arm, hand));
		autoChooser.addOption("SplineTest", new SplineTest(drive));
		SmartDashboard.putData(autoChooser);
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
		// drivebase bindings

		// arm bindings
		
		// hand bindings
		armController.leftBumper().onTrue(hand.startIntake());
		armController.leftBumper().onFalse(hand.stopIntake());
		armController.rightBumper().onTrue(hand.startOutput());
		armController.rightBumper().onFalse(hand.stopHand());

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return Autos.exampleAuto(m_exampleSubsystem);
		return autoChooser.getSelected();
	}
}
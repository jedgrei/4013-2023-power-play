// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	// ------------------------------- MEMBERS ------------------------------- //
	// subsystems
	private final SwerveDrive swerve = new SwerveDrive();

	// controllers + joystick limiters
	private final XboxController controller = new XboxController(0);
	private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);


	// ------------------------------- GENERAL ------------------------------- //
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		// m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	
	// ------------------------------- DISABLED ------------------------------ //
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	// ------------------------------ AUTONOMOUS ----------------------------- //
	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	// -------------------------------- TELEOP ------------------------------- //
	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		joystickDrive(true);
	}

	public void joystickDrive(boolean fieldRelative) {
		final double xSpeed = -xSpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02)) * SwerveDrive.maxSpeed;
		final double ySpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02)) * SwerveDrive.maxSpeed;
		final double rotSpeed = -ySpeedLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02)) * SwerveDrive.maxAngularSpeed;

		swerve.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
	}

	// --------------------------------- TEST -------------------------------- //
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	// ------------------------------ SIMUlATION ----------------------------- //
	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}

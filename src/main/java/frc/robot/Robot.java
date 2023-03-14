// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
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
	private final Arm arm = new Arm();

	// controllers + joystick limiters
	private final XboxController swerveController = new XboxController(1);
	private final XboxController armController = new XboxController(0);
	private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter elbowSpeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter wristSpeedLimiter = new SlewRateLimiter(3);


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
		double speedModSwerve = (swerveController.getLeftTriggerAxis() > 0.5 ? 1 : (swerveController.getRightTriggerAxis() > 0.5 ? 0.3 : 0.7));
		double speedModArm = (armController.getLeftTriggerAxis() > 0.5 ? 0.3 : (armController.getRightTriggerAxis() > 0.5 ? 0.1 : 0.2));
		// joystickDrive(true);
		// if(controller.getAButton()) {
		// 	swerve.driveForward();
		// }
		// if(controller.getBButton()) {
		// 	swerve.driveSpinToZero();
		// }
		// if(controller.getXButton()) {
		// 	swerve.stop();
		// }
		joystickDrive(false, speedModSwerve);
		joystickArmDrive(speedModArm);
		if(swerveController.getLeftBumper() && swerveController.getRightBumper() && swerveController.getXButton() && swerveController.getYButton() && swerveController.getBButton() && !swerveController.getAButton()) {
			swerve.resetSpinEncoders();
		}
	}

	public void joystickDrive(boolean fieldRelative, double speedMod) {
		final double xSpeed = speedMod * -xSpeedLimiter.calculate(MathUtil.applyDeadband(swerveController.getLeftY(), 0.05)) * SwerveDrive.maxSpeed;
		final double ySpeed = speedMod * -ySpeedLimiter.calculate(MathUtil.applyDeadband(swerveController.getLeftX(), 0.05)) * SwerveDrive.maxSpeed;
		final double rotSpeed = speedMod * -rotSpeedLimiter.calculate(MathUtil.applyDeadband(swerveController.getRightX(), 0.05)) * SwerveDrive.maxAngularSpeed;

		swerve.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
	}


	public void joystickArmDrive(double speedMod) {
		final double elbowSpeed = speedMod * -elbowSpeedLimiter.calculate(MathUtil.applyDeadband(armController.getLeftY(), 0.05)) * SwerveDrive.maxSpeed;
		final double wristSpeed = speedMod * -wristSpeedLimiter.calculate(MathUtil.applyDeadband(armController.getRightY(), 0.05)) * SwerveDrive.maxSpeed;

		arm.drive(elbowSpeed, wristSpeed);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAlwaysPid;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Hand;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class TankRobot extends TimedRobot {
	// ------------------------------- MEMBERS ------------------------------- //
	// subsystems
	private final DriveBase drive = new DriveBase();
	// private final Arm arm = new Arm();
	private final ArmAlwaysPid arm = new ArmAlwaysPid();
	// private final Hand hand = new Hand();

	// controllers + joystick limiters
	private final XboxController driveController = new XboxController(1);
	private final XboxController armController = new XboxController(0);
	private final CommandXboxController carmController = new CommandXboxController(0);

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

	private boolean holding;

	@Override
	public void teleopPeriodic() {
		double speedModArm = (armController.getLeftTriggerAxis() > 0.5 ? 12 : (armController.getRightTriggerAxis() > 0.5 ? 6 : 9));

		joystickBaseDrive();
		joystickArmDrive(speedModArm);

		if(armController.getLeftBumper()) {
			arm.setHandSpeed(-1);
			SmartDashboard.putString("hand", "in");
			holding = true;
		}
		else if(armController.getRightBumper()) {
			arm.setHandSpeed(1);
			SmartDashboard.putString("hand", "out");
			holding = false;
		}
		else {
			// if(holding) {
			// 	arm.setHandSpeed(-0.1);
			// 	SmartDashboard.putString("hand", "holding");
			// }
			// else {
				arm.stopHand();
				SmartDashboard.putString("hand", "stopped");
			// }
		}
	}

	public void joystickBaseDrive() {
		// fast mode
		if(driveController.getLeftTriggerAxis() > 0.5) {
			SmartDashboard.putString("basemode", "fast");
			drive.drive(
				sgnsqr(driveController.getLeftY()) * drive.forwardSpeedFast,
				sgnsqr(driveController.getRightX()) * drive.rotSpeedFast
			);
		}
		// slow mode
		else if(driveController.getRightTriggerAxis() > 0.5) {
			SmartDashboard.putString("basemode", "slow");
			drive.drive(
				sgnsqr(driveController.getLeftY()) * drive.forwardSpeedSlow,
				sgnsqr(driveController.getRightX()) * drive.rotSpeedSlow
			);
		}
		// normal
		else {
			SmartDashboard.putString("basemode", "norm");	
			drive.drive(
				sgnsqr(driveController.getLeftY()) * drive.forwardSpeed,
				sgnsqr(driveController.getRightX()) * drive.rotSpeed
			);
		}
		SmartDashboard.putNumber("forinp", driveController.getLeftY());
		SmartDashboard.putNumber("rotinp", driveController.getRightX());
	}


	public void joystickArmDrive(double speedMod) {
		// fast mode
		if(armController.getLeftTriggerAxis() > 0.5) {
			arm.drive(
				sgnsqr(armController.getLeftY()) * arm.elbowSpeedFast,
				sgnsqr(armController.getRightY()) * arm.wristSpeedFast
			);
		}
		// slow mode
		else if(armController.getRightTriggerAxis() > 0.5) {
			arm.drive(
				sgnsqr(armController.getLeftY()) * arm.elbowSpeedSlow,
				sgnsqr(armController.getRightY()) * arm.wristSpeedSlow
			);
		}
		// normal
		else {
			arm.drive(
				sgnsqr(armController.getLeftY()) * arm.elbowSpeed,
				sgnsqr(armController.getRightY()) * arm.wristSpeed
			);
		}
		arm.updateVelocityController();
	}

	// square a number while keeping its sign
	public double sgnsqr(double n) {
		return n * Math.abs(n);
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

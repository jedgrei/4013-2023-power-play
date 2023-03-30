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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAlwaysPid;
import frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.trajectory.constraint.*;

import java.util.List;


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


	// controllers + joystick limiters
	private final XboxController driveController = new XboxController(1);
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
	public void autonomousInit() {
		CommandScheduler.getInstance().schedule(getAutonomousCommand());
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

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
			if(holding) {
				arm.setHandSpeed(-0.1);
				SmartDashboard.putString("hand", "holding");
			}
			else {
				arm.stopHand();
				SmartDashboard.putString("hand", "stopped");
			}
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

	public Command getAutonomousCommand() {
		// Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(
					drive.ksVolts,
					drive.kvVoltSecondsPerMeter,
					drive.kaVoltSecondsSquaredPerMeter),
				drive.kinematics,
				10);
	
		// Create config for trajectory
		TrajectoryConfig config =
			new TrajectoryConfig(
					drive.kMaxSpeedMetersPerSecond,
					drive.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(drive.kinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);
	
		// An example trajectory to follow.  All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			// Pass config
			config
		);
	
		RamseteCommand ramseteCommand = new RamseteCommand(
			exampleTrajectory,
			drive::getPose,
			new RamseteController(drive.kRamseteB, drive.kRamseteZeta),
			new SimpleMotorFeedforward(drive.ksVolts, drive.kvVoltSecondsPerMeter, drive.kaVoltSecondsSquaredPerMeter),
			drive.kinematics,
			drive::getWheelSpeeds,
			new PIDController(drive.kPDriveVel, 0, 0),
			new PIDController(drive.kPDriveVel, 0, 0),
			drive::tankDriveVolts,
			drive
		);
	
		// Reset odometry to the starting pose of the trajectory.
		drive.resetOdometry(exampleTrajectory.getInitialPose());
	
		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
	}
}

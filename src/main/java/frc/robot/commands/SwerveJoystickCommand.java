// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveJoystickCommand extends CommandBase {
	private final SwerveDrive swerveDrive;
	private final Supplier<Double> xSpeedFunction, ySpeedFunction, rotSpeedFunction;
	private final Supplier<Boolean> fieldOrientedFunction;
	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public SwerveJoystickCommand(SwerveDrive swerveDrive,
			Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> rotSpeedFunction,
			Supplier<Boolean> fieldOrientedFunction) {
		this.swerveDrive = swerveDrive;
		this.xSpeedFunction = xSpeedFunction;
		this.ySpeedFunction = ySpeedFunction;
		this.rotSpeedFunction = rotSpeedFunction;
		this.fieldOrientedFunction = fieldOrientedFunction;
		this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
		this.rotLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
		addRequirements(swerveDrive);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// get input
		double xSpeed = xSpeedFunction.get();
		double ySpeed = ySpeedFunction.get();
		double rotSpeed = rotSpeedFunction.get();

		// add deadband
		xSpeed = Math.abs(xSpeed) > InputConstants.kDeadband ? xSpeed : 0.0;
		ySpeed = Math.abs(ySpeed) > InputConstants.kDeadband ? ySpeed : 0.0;
		rotSpeed = Math.abs(rotSpeed) > InputConstants.kDeadband ? rotSpeed : 0.0;

		// smooth driving
		xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
		rotSpeed = rotLimiter.calculate(rotSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
	
		// construct chassis speeds
		ChassisSpeeds chassisSpeeds;
		if (fieldOrientedFunction.get()) {
			// relative to field
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveDrive.getRotation2d());
		}
		else {
			// relative to robot
			chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
		}

		// convert chassis speeds into module states
		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		// output module states to wheels
		swerveDrive.setModuleStates(moduleStates);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		swerveDrive.stopModules();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

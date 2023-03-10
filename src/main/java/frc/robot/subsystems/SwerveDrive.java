// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds
	public static final double maxSpeed = 3.0;
	public static final double maxAngularSpeed = Math.PI;

	// wheel locations
	public static final Translation2d frontLeftLoc = new Translation2d();
	public static final Translation2d frontRightLoc = new Translation2d();
	public static final Translation2d rearRightLoc = new Translation2d();
	public static final Translation2d rearLeftLoc = new Translation2d();

	// wheel ports
	public static final int flPowerPort = 0;
	public static final int flSpinPort = 0;
	public static final int frPowerPort = 0;
	public static final int frSpinPort = 0;
	public static final int rrPowerPort = 0;
	public static final int rrSpinPort = 0;
	public static final int rlPowerPort = 0;
	public static final int rlSpinPort = 0;

	// ------------------------------- MEMBERS ------------------------------- //
	// wheel modules
	SwerveWheel frontLeft = new SwerveWheel(flPowerPort, flSpinPort);
	SwerveWheel frontRight = new SwerveWheel(frPowerPort, frSpinPort);
	SwerveWheel rearRight = new SwerveWheel(rrPowerPort, rrSpinPort);
	SwerveWheel rearLeft = new SwerveWheel(rlPowerPort, rlSpinPort);

	// imu
    ADIS16448_IMU imu;

	// odometry + kinematics
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		frontLeftLoc,
		frontRightLoc,
		rearRightLoc,
		rearLeftLoc
	);
	private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
		kinematics,
		new Rotation2d(imu.getAngle()),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition(),
		}
	);

	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public SwerveDrive() {
		imu.reset();
	}

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		
	}

	// ------------------------------- METHODS ------------------------------- //
	
	public Rotation2d getHeading() {
		return new Rotation2d(imu.getAngle());
	}

	public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(
			fieldRelative ?
			ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getHeading()) :
			new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
		);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
		frontLeft.setDesiredState(states[0]);
		frontRight.setDesiredState(states[1]);
		rearRight.setDesiredState(states[2]);
		rearLeft.setDesiredState(states[3]);
	}
}

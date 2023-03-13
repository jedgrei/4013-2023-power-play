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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds
	public static final double maxSpeed = 3.0;
	public static final double maxAngularSpeed = Math.PI;

	// wheel ports
	public static final int flPowerPort = 4;
	public static final int flSpinPort = 13;
	public static final int flSpinEncPort = 1;

	public static final int frPowerPort = 1;
	public static final int frSpinPort = 8;
	public static final int frSpinEncPort = 0;

	public static final int rrPowerPort = 11;
	public static final int rrSpinPort = 5;
	public static final int rrSpinEncPort = 2;

	public static final int rlPowerPort = 12;
	public static final int rlSpinPort = 10;
	public static final int rlSpinEncPort = 3;
	// wheel locations
	public static final double wheelDist = Units.inchesToMeters(22.83);
	public static final Translation2d frontLeftLoc = new Translation2d(+wheelDist/2, +wheelDist/2);
	public static final Translation2d frontRightLoc = new Translation2d(+wheelDist/2, -wheelDist/2);
	public static final Translation2d rearRightLoc = new Translation2d(-wheelDist/2, -wheelDist/2);
	public static final Translation2d rearLeftLoc = new Translation2d(-wheelDist/2, +wheelDist/2);

	// odometry + kinematics
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		frontLeftLoc,
		frontRightLoc,
		rearRightLoc,
		rearLeftLoc
	);
	// ------------------------------- MEMBERS ------------------------------- //
	// wheel modules
	SwerveWheel frontLeft = new SwerveWheel(flPowerPort, flSpinPort, flSpinEncPort, 0);
	SwerveWheel frontRight = new SwerveWheel(frPowerPort, frSpinPort, flSpinEncPort, 0);
	SwerveWheel rearRight = new SwerveWheel(rrPowerPort, rrSpinPort, flSpinEncPort, 0);
	SwerveWheel rearLeft = new SwerveWheel(rlPowerPort, rlSpinPort, flSpinEncPort, 0);

	// imu
    ADIS16448_IMU imu;

	private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
		kinematics,
		getHeading(),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearRight.getPosition(),
			rearLeft.getPosition(),
		}
	);


	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public SwerveDrive() {
		imu.reset();
	}

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		updateOdometry();
	}

	// ------------------------------- METHODS ------------------------------- //
	
	public Rotation2d getHeading() {
		return new Rotation2d(imu.getAngle());
	}

	public SwerveModulePosition[] getPositions() {
		return new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearRight.getPosition(),
			rearLeft.getPosition(),
		};
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

	public void updateOdometry() {
		odometry.update(
			getHeading(),
			getPositions()
		);
	}
}

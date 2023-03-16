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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	public static final int rrSpinPort = 19;
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
	SwerveWheel frontRight = new SwerveWheel(frPowerPort, frSpinPort, frSpinEncPort, Math.PI/2);
	SwerveWheel rearRight = new SwerveWheel(rrPowerPort, rrSpinPort, rrSpinEncPort, Math.PI);
	SwerveWheel rearLeft = new SwerveWheel(rlPowerPort, rlSpinPort, rlSpinEncPort, 3*Math.PI/2);
	SwerveWheel frontLeft = new SwerveWheel(flPowerPort, flSpinPort, flSpinEncPort, 0);

	// imu
    ADIS16448_IMU imu = new ADIS16448_IMU();

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

		SmartDashboard.putNumber("tflp", states[0].speedMetersPerSecond);
		SmartDashboard.putNumber("tfrp", states[1].speedMetersPerSecond);
		SmartDashboard.putNumber("trrp", states[2].speedMetersPerSecond);
		SmartDashboard.putNumber("trlp", states[3].speedMetersPerSecond);
		SmartDashboard.putNumber("tfls", states[0].angle.getDegrees());
		SmartDashboard.putNumber("tfrs", states[1].angle.getDegrees());
		SmartDashboard.putNumber("trrs", states[2].angle.getDegrees());
		SmartDashboard.putNumber("trls", states[3].angle.getDegrees());

		SmartDashboard.putNumber("aflp", frontLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("afrp", frontRight.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("arrp", rearRight.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("arlp", rearLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("afls", frontLeft.getState().angle.getDegrees());
		SmartDashboard.putNumber("afrs", frontRight.getState().angle.getDegrees());
		SmartDashboard.putNumber("arrs", rearRight.getState().angle.getDegrees());
		SmartDashboard.putNumber("arls", rearLeft.getState().angle.getDegrees());
	}

	public void drive(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
		frontLeft.setDesiredState(states[0]);
		frontRight.setDesiredState(states[1]);
		rearRight.setDesiredState(states[2]);
		rearLeft.setDesiredState(states[3]);

		SmartDashboard.putNumber("tflp", states[0].speedMetersPerSecond);
		SmartDashboard.putNumber("tfrp", states[1].speedMetersPerSecond);
		SmartDashboard.putNumber("trrp", states[2].speedMetersPerSecond);
		SmartDashboard.putNumber("trlp", states[3].speedMetersPerSecond);
		SmartDashboard.putNumber("tfls", states[0].angle.getDegrees() % 360);
		SmartDashboard.putNumber("tfrs", states[1].angle.getDegrees() % 360);
		SmartDashboard.putNumber("trrs", states[2].angle.getDegrees() % 360);
		SmartDashboard.putNumber("trls", states[3].angle.getDegrees() % 360);

		SmartDashboard.putNumber("aflp", frontLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("afrp", frontRight.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("arrp", rearRight.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("arlp", rearLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("afls", frontLeft.getState().angle.getDegrees() % 360);
		SmartDashboard.putNumber("afrs", frontRight.getState().angle.getDegrees() % 360);
		SmartDashboard.putNumber("arrs", rearRight.getState().angle.getDegrees() % 360);
		SmartDashboard.putNumber("arls", rearLeft.getState().angle.getDegrees() % 360);
	}

	public void stop() {
		frontLeft.stop();
		frontRight.stop();
		rearRight.stop();
		rearLeft.stop();
	}

	public void driveSpinToZero() {
		// frontLeft.setDesiredState(new SwerveModuleState(
		// 	0, new Rotation2d(0)
		// ));
		// frontRight.setDesiredState(new SwerveModuleState(
		// 	0, new Rotation2d(0)
		// ));
		// rearRight.setDesiredState(new SwerveModuleState(
		// 	0, new Rotation2d(0)
		// ));
		// rearLeft.setDesiredState(new SwerveModuleState(
		// 	0, new Rotation2d(0)
		// ));
		SwerveModuleState[] states = new SwerveModuleState[] {
			new SwerveModuleState(0, new Rotation2d(0)),
			new SwerveModuleState(0, new Rotation2d(0)),
			new SwerveModuleState(0, new Rotation2d(0)),
			new SwerveModuleState(0, new Rotation2d(0))
		};
		drive(states);
	}

	public void driveForward() {
		SwerveModuleState[] states = new SwerveModuleState[] {
			new SwerveModuleState(1, new Rotation2d(0)),
			new SwerveModuleState(1, new Rotation2d(0)),
			new SwerveModuleState(1, new Rotation2d(0)),
			new SwerveModuleState(1, new Rotation2d(0))
		};
		drive(states);
	}

	public void updateOdometry() {
		odometry.update(
			getHeading(),
			getPositions()
		);
	}

	public void resetSpinEncoders() {
		frontLeft.resetSpinEncoder();
		frontRight.resetSpinEncoder();
		rearRight.resetSpinEncoder();
		rearLeft.resetSpinEncoder();
	}

	// auto

	public void autoTranslate(double forward, double strafe) {
		
	}
}

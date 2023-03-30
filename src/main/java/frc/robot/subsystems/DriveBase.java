// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.relation.Relation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

public class DriveBase extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds
	public final double forwardSpeed = 3.0;
	public final double rotSpeed = 3.0;
	public final double forwardSpeedFast = 5.0;
	public final double rotSpeedFast = 5.0;
	public final double forwardSpeedSlow = 1.0;
	public final double rotSpeedSlow = 1.0;

	public final double ksVolts = 0.22;
    public final double kvVoltSecondsPerMeter = 1.98;
    public final double kaVoltSecondsSquaredPerMeter = 0.2;

    public final double kPDriveVel = 8.5;

	public final double kMaxSpeedMetersPerSecond = 3;
    public final double kMaxAccelerationMetersPerSecondSquared = 1;

	public final double kRamseteB = 2;
    public final double kRamseteZeta = 0.7;

	// ports
    public final int l0Port = 2;
    public final int l1Port = 4;
    public final int r0Port = 3;
    public final int r1Port = 5;

	// wheel locations


	// ------------------------------- MEMBERS ------------------------------- //

	// imu
    CANSparkMax left0Motor = new CANSparkMax(l0Port, MotorType.kBrushless);
    CANSparkMax left1Motor = new CANSparkMax(l1Port, MotorType.kBrushless);
	
	CANSparkMax right0Motor = new CANSparkMax(r0Port, MotorType.kBrushless);
    CANSparkMax right1Motor = new CANSparkMax(r1Port, MotorType.kBrushless);

	RelativeEncoder leftEncoder;
	RelativeEncoder rightEncoder;

	MotorControllerGroup leftMotors;
	MotorControllerGroup rightMotors;

	DifferentialDrive diffDrive;

    ADIS16448_IMU imu = new ADIS16448_IMU();

	Rotation2d heading = new Rotation2d(imu.getAngle());
	Pose2d position = new Pose2d(0, 0, heading);

	public DifferentialDriveKinematics kinematics;
	public DifferentialDriveOdometry odometry;

	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public DriveBase() {
		left0Motor.setInverted(false);
		left1Motor.setInverted(true);
		right0Motor.setInverted(true);
		right1Motor.setInverted(false);
		
		leftMotors = new MotorControllerGroup(left0Motor, left1Motor);
		rightMotors = new MotorControllerGroup(left0Motor, left1Motor);

		leftEncoder = left0Motor.getEncoder();
		rightEncoder = right0Motor.getEncoder();

		leftEncoder.setPositionConversionFactor(1 / 10.71);
		rightEncoder.setPositionConversionFactor(1 / 10.71);

		leftEncoder.setVelocityConversionFactor(1 / 10.71);
		rightEncoder.setVelocityConversionFactor(1 / 10.71);

		diffDrive = new DifferentialDrive(leftMotors, rightMotors);

		kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22.0));
		odometry = new DifferentialDriveOdometry(heading, getEncoderDistance(leftEncoder), getEncoderDistance(rightEncoder), position);
	}

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		heading = new Rotation2d(imu.getAngle());
		position = odometry.update(heading, getEncoderDistance(leftEncoder), getEncoderDistance(rightEncoder));
	}

	// ------------------------------- METHODS ------------------------------- //

	public void drive(double forSpeed, double rotSpeed) {
		diffDrive.arcadeDrive(forSpeed, rotSpeed, false);
		
		SmartDashboard.putNumber("forspeed", forSpeed);
		SmartDashboard.putNumber("rotspeed", rotSpeed);
		SmartDashboard.putNumber("leftspeed", forSpeed + rotSpeed);
		SmartDashboard.putNumber("rightspeed", forSpeed - rotSpeed);
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
	}
	
	public void resetEncoders() {
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(heading, getEncoderDistance(leftEncoder), getEncoderDistance(rightEncoder), pose);
	}

	public void zeroHeading() {
		imu.reset();
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public double getEncoderDistance(RelativeEncoder encoder) {
		return encoder.getPosition() * Units.inchesToMeters(6 * Math.PI);
	}

	public double getAverageEncoderDistance() {
		return (getEncoderDistance(leftEncoder) + getEncoderDistance(rightEncoder)) / 2.0;
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftMotors.setVoltage(leftVolts);
		rightMotors.setVoltage(rightVolts);
		diffDrive.feed();
	}
}

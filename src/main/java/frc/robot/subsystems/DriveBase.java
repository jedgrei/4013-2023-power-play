// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds
	public static final double forwardSpeed = 3.0;
	public static final double rotSpeed = 3.0;
	public static final double forwardSpeedFast = 5.0;
	public static final double rotSpeedFast = 5.0;
	public static final double forwardSpeedSlow = 1.0;
	public static final double rotSpeedSlow = 1.0;

	public static final double maxSpeed = 5.0;
	public static final double maxAngularSpeed = Math.PI;

	// ports
    public static final int l0Port = 2;
    public static final int l1Port = 4;
    public static final int r0Port = 3;
    public static final int r1Port = 5;

	// controller vals
	public static final double kpLeft = 1, kiLeft = 0, kdLeft = 0;
	public static final double kpRight = 1, kiRight = 0, kdRight = 0;

	// wheel locations


	// ------------------------------- MEMBERS ------------------------------- //

	// imu
    CANSparkMax left0Motor = new CANSparkMax(l0Port, MotorType.kBrushless);
    CANSparkMax left1Motor = new CANSparkMax(l1Port, MotorType.kBrushless);
    MotorControllerGroup leftMotors = new MotorControllerGroup(left0Motor, left1Motor);
	
	CANSparkMax right0Motor = new CANSparkMax(r0Port, MotorType.kBrushless);
    CANSparkMax right1Motor = new CANSparkMax(r1Port, MotorType.kBrushless);
    MotorControllerGroup rightMotors = new MotorControllerGroup(right0Motor, right1Motor);

	DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

    ADIS16448_IMU imu = new ADIS16448_IMU();

	private PIDController leftPid = new PIDController(kpLeft, kiLeft, kdLeft);
	private PIDController rightPid = new PIDController(kpRight, kiRight, kdRight);

	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public DriveBase() {
		rightMotors.setInverted(true);

		left0Motor.setSmartCurrentLimit(40);
		left1Motor.setSmartCurrentLimit(40);
		right0Motor.setSmartCurrentLimit(40);
		right1Motor.setSmartCurrentLimit(40);

		// left0Motor.setIdleMode(IdleMode.kCoast);
		// left1Motor.setIdleMode(IdleMode.kCoast);
		// right0Motor.setIdleMode(IdleMode.kCoast);
		// right1Motor.setIdleMode(IdleMode.kCoast);
	}

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {

	}

	// ------------------------------- METHODS ------------------------------- //
	
	public Rotation2d getHeading() {
		return new Rotation2d(imu.getAngle());
	}

	public void drive(double forSpeed, double rotSpeed) {
		// diffDrive.arcadeDrive(forSpeed, rotSpeed, false);
		left0Motor.set(forSpeed + rotSpeed);
		left1Motor.set(forSpeed + rotSpeed);
		right0Motor.set(-forSpeed + rotSpeed);
		right1Motor.set(-forSpeed + rotSpeed);
		SmartDashboard.putNumber("forspeed", forSpeed);
		SmartDashboard.putNumber("rotspeed", rotSpeed);
		SmartDashboard.putNumber("leftspeed", forSpeed + rotSpeed);
		SmartDashboard.putNumber("rightspeed", forSpeed - rotSpeed);
	}

	public void driveToPosition(double leftPos, double rightPos) {
		
	}

	public void updatePid() {

	}

	// ------------------------------- COMMANDS ------------------------------ //
	// public CommandBase driveToPositionCommand(double leftPos, double rightPos) {
		
	// }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds
	public static final double maxAngularVelocityElbow = Math.PI / 8;
	public static final double maxAngularVelocityWrist = Math.PI / 8;
	public static final double maxAngularAccelerationElbow = Math.PI / 8;
	public static final double maxAngularAccelerationWrist = Math.PI / 8;
    public static final double maxHandVelocity = 1;
	public static final int elbowCurrentLimit = 25;
	public static final int wristCurrentLimit = 35;

	// motor/encoder ports
    // TODO: find arm ports
	public static final int elbow0Port = 16;
	public static final int elbow1Port = 9;
	public static final int elbow2Port = 3;
	public static final int elbow3Port = 2;
	public static final int wrist0Port = 6;
	public static final int wrist1Port = 5;
    public static final int handPort = 17;

    // contoller constants TODO: tune nums
    private static final double kpElbow = 1, kiElbow = 0, kdElbow = 0;
	private static final double kpWrist = 1, kiWrist = 0, kdWrist = 0;
	private static final double ksElbow = 1, kgElbow = 0, kvElbow = 0.5;
	private static final double ksWrist = 1, kgWrist = 0, kvWrist = 0.5;

	// ------------------------------- MEMBERS ------------------------------- //
	// motors + encoders
    CANSparkMax elbow0Motor, elbow1Motor, elbow2Motor, elbow3Motor;
	CANSparkMax wrist1Motor, handMotor;
	AbsoluteEncoder elbowEncoder;
	RelativeEncoder wristEncoder;

	boolean velocity;

    // pid controllers + feedforward
    private final ProfiledPIDController elbowPositionPid = new ProfiledPIDController(
		kpElbow, kiElbow, kdElbow,
		new TrapezoidProfile.Constraints(
			maxAngularVelocityElbow, maxAngularAccelerationElbow
		)
	);
    private final ProfiledPIDController wristPositionPid = new ProfiledPIDController(
		kpWrist, kiWrist, kdWrist,
		new TrapezoidProfile.Constraints(
			maxAngularVelocityWrist, maxAngularAccelerationWrist
		)
	);
    private final ProfiledPIDController elbowVelocityPid = new ProfiledPIDController(
		kpElbow, kiElbow, kdElbow,
		new TrapezoidProfile.Constraints(
			maxAngularVelocityElbow, maxAngularAccelerationElbow
		)
	);
    private final ProfiledPIDController wristVelocityPid = new ProfiledPIDController(
		kpWrist, kiWrist, kdWrist,
		new TrapezoidProfile.Constraints(
			maxAngularVelocityWrist, maxAngularAccelerationWrist
		)
	);

	private final ArmFeedforward elbowFeedforward = new ArmFeedforward(ksElbow, kgElbow, kvElbow);
	private final ArmFeedforward wristFeedforward = new ArmFeedforward(ksWrist, kgWrist, kvWrist);

	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public Arm() {
        elbow0Motor = new CANSparkMax(elbow0Port, MotorType.kBrushed);
        elbow1Motor = new CANSparkMax(elbow1Port, MotorType.kBrushed);
        elbow2Motor = new CANSparkMax(elbow2Port, MotorType.kBrushed);
        elbow3Motor = new CANSparkMax(elbow3Port, MotorType.kBrushed);
        // wrist0Motor = new CANSparkMax(wrist0Port, MotorType.kBrushless);
        wrist1Motor = new CANSparkMax(wrist1Port, MotorType.kBrushless);
		handMotor = new CANSparkMax(handPort, MotorType.kBrushless);
	
		elbow0Motor.setSmartCurrentLimit(elbowCurrentLimit);
		elbow1Motor.setSmartCurrentLimit(elbowCurrentLimit);
		elbow2Motor.setSmartCurrentLimit(elbowCurrentLimit);
		elbow3Motor.setSmartCurrentLimit(elbowCurrentLimit);
		wrist1Motor.setSmartCurrentLimit(wristCurrentLimit);

		// elbow0Motor.setInverted(true);
		// elbow2Motor.setInverted(true);
		// wrist1Motor.setInverted(true);

        elbowEncoder = elbow0Motor.getAbsoluteEncoder(Type.kDutyCycle);
		// elbowEncoder = elbow0Motor.getEncoder();
		wristEncoder = wrist1Motor.getEncoder();
		// wristEncoder = wrist0Motor.getEncoder();
        // wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // elbowEncoder.setPositionConversionFactor(1/35.5);
        // wristEncoder.setPositionConversionFactor(1/35.5);
        // wristEncoder.setPositionConversionFactor(1/35.5);
		elbowEncoder.setPositionConversionFactor((2*Math.PI)/(42*48));
		wristEncoder.setPositionConversionFactor((2*Math.PI)/(42*48));
		elbowEncoder.setVelocityConversionFactor((2*Math.PI)/(42*48));
		elbowEncoder.setVelocityConversionFactor((2*Math.PI)/(42*48));

		targetElbowPosition = 0;
		targetWristPosition = 0;
		targetElbowVelocity = 0;
		targetWristVelocity = 0;

		velocity = false;
    }

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		updateControllers();
		SmartDashboard.putNumber("elbow angle", elbowEncoder.getPosition());
		SmartDashboard.putNumber("wrist angle", wristEncoder.getPosition());
	
		// if(targetElbowVelocity == 0 && targetWristVelocity == 0) {
		// 	velocity = false;
		// }
	}

	public void updateControllers() {
		if (velocity) {
			double elbowOutput = elbowVelocityPid.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
			double elbowFf = elbowFeedforward.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
        	double wristOutput = wristVelocityPid.calculate(wristEncoder.getPosition(), targetWristVelocity);
			double wristFf = wristFeedforward.calculate(wristEncoder.getPosition(), targetWristVelocity);

			// elbow0Motor.set(-elbowOutput - elbowFf);
			// elbow1Motor.set(elbowOutput + elbowFf);
			// elbow2Motor.set(elbowOutput + elbowFf);
			// elbow3Motor.set(-elbowOutput - elbowFf);
			// wrist1Motor.set(wristOutput + wristFf);
			
			elbow0Motor.setVoltage(-elbowOutput);
			elbow1Motor.setVoltage(elbowOutput);
			elbow2Motor.setVoltage(elbowOutput);
			elbow3Motor.setVoltage(-elbowOutput);
			wrist1Motor.setVoltage(wristOutput);

			SmartDashboard.putNumber("elbow-v", elbowOutput);
			SmartDashboard.putNumber("wrist-v", wristOutput);
			SmartDashboard.putNumber("target-ev", targetElbowVelocity);
			SmartDashboard.putNumber("target-wv", targetWristVelocity);
		}
		else {
			double elbowOutput = elbowPositionPid.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
			double elbowFf = elbowFeedforward.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
        	double wristOutput = wristPositionPid.calculate(wristEncoder.getPosition(), targetWristVelocity);
			double wristFf = wristFeedforward.calculate(wristEncoder.getPosition(), targetWristVelocity);
		
			elbow0Motor.setVoltage(-elbowOutput);
			elbow1Motor.setVoltage(elbowOutput);
			elbow2Motor.setVoltage(elbowOutput);
			elbow3Motor.setVoltage(-elbowOutput);
			wrist1Motor.setVoltage(wristOutput);
			
			SmartDashboard.putNumber("elbow-p", elbowOutput);
			SmartDashboard.putNumber("wrist-p", wristOutput);
			SmartDashboard.putNumber("target-ep", targetElbowPosition);
			SmartDashboard.putNumber("target-wv", targetWristPosition);
		}
	}
	
	// ------------------------------- METHODS ------------------------------- //
    public void drive(double eSpeed, double wSpeed) {
		velocity = true;
		targetElbowVelocity = eSpeed;
		targetWristVelocity = wSpeed;
	}

	public void driveToPosition(double ePosition, double wPosition) {
		velocity = false;
		targetElbowPosition = ePosition;
		targetWristPosition = wPosition;
	}

    public void setHandSpeed(double speed) {
        handMotor.set(speed);
    }

    public void stopHand() {
        handMotor.set(0);
    }

	double targetElbowPosition;
	double targetWristPosition;
	double targetElbowVelocity;
	double targetWristVelocity;

	// double elbowOutput;
	// double elbowFf;
	// double wristOutput;
	// double wristFf;

	// ------------------------------- COMMANDS ------------------------------ //
	public CommandBase setHandSpeedCommand(double speed) {
		return this.runOnce(() -> handMotor.set(speed));
	}

	public CommandBase driveArmCommand(double eSpeed, double wSpeed) {
		return this.run(() -> drive(eSpeed, wSpeed));
	}
}

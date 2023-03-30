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
	public static final double elbowSpeed = 3.0;
	public static final double wristSpeed = 9.0;
	public static final double elbowSpeedFast = 5.0;
	public static final double wristSpeedFast = 12.0;
	public static final double elbowSpeedSlow = 1.0;
	public static final double wristSpeedSlow = 6.0;

	public static final double maxAngularVelocityElbow = 3;
	public static final double maxAngularVelocityWrist = 3;
	public static final double maxAngularAccelerationElbow = 3;
	public static final double maxAngularAccelerationWrist = 3;
    public static final double maxHandVelocity = 1;
	public static final int elbowCurrentLimit = 25;
	public static final int wristCurrentLimit = 35;

	// motor/encoder ports
    // TODO: find arm ports
	public static final int elbow0Port = 6; //cim4
	public static final int elbow1Port = 7; //cim1
	public static final int wristPort = 8; //arm1
    public static final int handPort = 17;



    // contoller constants TODO: tune nums
    private static final double kpElbow = 1, kiElbow = 0, kdElbow = 0;
	private static final double kpWrist = 0.4, kiWrist = 0, kdWrist = 0;
	private static final double ksElbow = 1, kgElbow = 0, kvElbow = 0.5;
	private static final double ksWrist = 1, kgWrist = 0, kvWrist = 0.5;

	// ------------------------------- MEMBERS ------------------------------- //
	// motors + encoders
    CANSparkMax elbow0Motor, elbow1Motor;
	CANSparkMax wristMotor, handMotor;
	AbsoluteEncoder elbowEncoder;
	AbsoluteEncoder wristEncoder;

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
        wristMotor = new CANSparkMax(wristPort, MotorType.kBrushless);
		handMotor = new CANSparkMax(handPort, MotorType.kBrushless);
	
		// elbow0Motor.setSmartCurrentLimit(elbowCurrentLimit);
		// elbow1Motor.setSmartCurrentLimit(elbowCurrentLimit);
		// wristMotor.setSmartCurrentLimit(wristCurrentLimit);

		elbow0Motor.setInverted(false);
		elbow1Motor.setInverted(true);
		wristMotor.setInverted(false);
		handMotor.setInverted(false);

        elbowEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
		wristEncoder = handMotor.getAbsoluteEncoder(Type.kDutyCycle);

		targetElbowPosition = 0;
		targetWristPosition = 0;
		targetElbowVelocity = 0;
		targetWristVelocity = 0;

		velocity = true;
		// elbowEncoder.setPositionConversionFactor(2 * Math.PI);
		// wristEncoder.setPositionConversionFactor(2 * Math.PI);
		// elbowEncoder.setVelocityConversionFactor(2 * Math.PI);
		// wristEncoder.setVelocityConversionFactor(2 * Math.PI);
    }

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		// updateControllers();
		SmartDashboard.putNumber("elbow angle", elbowEncoder.getPosition());
		SmartDashboard.putNumber("wrist angle", wristEncoder.getPosition());
	}

	double targetElbowPosition;
	double targetWristPosition;
	double targetElbowVelocity;
	double targetWristVelocity;
	
	public void updateControllers() {
		if (velocity) {
			double elbowOutput = elbowVelocityPid.calculate(elbowEncoder.getVelocity(), targetElbowVelocity);
			// double elbowFf = elbowFeedforward.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
        	double wristOutput = wristVelocityPid.calculate(wristEncoder.getVelocity(), targetWristVelocity);
			// double wristFf = wristFeedforward.calculate(wristEncoder.getPosition(), targetWristVelocity);
			
			elbow0Motor.setVoltage(elbowOutput);
			elbow1Motor.setVoltage(elbowOutput);
			wristMotor.setVoltage(wristOutput);

			SmartDashboard.putNumber("elbow-v", elbowOutput);
			SmartDashboard.putNumber("wrist-v", wristOutput);
			SmartDashboard.putNumber("target-ev", targetElbowVelocity);
			SmartDashboard.putNumber("target-wv", targetWristVelocity);
		}
		else {
			double elbowOutput = elbowPositionPid.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
			// double elbowFf = elbowFeedforward.calculate(elbowEncoder.getPosition(), targetElbowVelocity);
        	double wristOutput = wristPositionPid.calculate(wristEncoder.getPosition(), targetWristVelocity);
			// double wristFf = wristFeedforward.calculate(wristEncoder.getPosition(), targetWristVelocity);
		
			elbow0Motor.setVoltage(elbowOutput);
			elbow1Motor.setVoltage(elbowOutput);
			wristMotor.setVoltage(wristOutput);
			
			SmartDashboard.putNumber("elbow-p", elbowOutput);
			SmartDashboard.putNumber("wrist-p", wristOutput);
			SmartDashboard.putNumber("target-ep", targetElbowPosition);
			SmartDashboard.putNumber("target-wp", targetWristPosition);
		}
	}
	
	// ------------------------------- METHODS ------------------------------- //
	public void drive(double eSpeed, double wSpeed) {
		elbow0Motor.setVoltage(eSpeed);
		elbow1Motor.setVoltage(eSpeed);
		wristMotor.setVoltage(wSpeed);
		
		SmartDashboard.putNumber("elbow-v", eSpeed);
		SmartDashboard.putNumber("wrist-v", wSpeed);
		SmartDashboard.putNumber("elbow-angle", elbowEncoder.getPosition());
		SmartDashboard.putNumber("wrist-angle", wristEncoder.getPosition());
		SmartDashboard.putNumber("target-ev", eSpeed);
		SmartDashboard.putNumber("target-wv", wSpeed);
	}

	public void driveToPosition(double ePosition, double wPosition) {
		velocity = false;
		targetElbowPosition = ePosition;
		targetWristPosition = wPosition;
	}

	public void updatePid() {

	}

	public void stop() {
		drive(0, 0);
		elbow0Motor.set(0);
		elbow1Motor.set(0);
		wristMotor.set(0);
	}

    public void setHandSpeed(double speed) {
        handMotor.set(speed);
    }

    public void stopHand() {
        handMotor.set(0);
    }

	// ------------------------------- COMMANDS ------------------------------ //
	public CommandBase setHandSpeedCommand(double speed) {
		return this.runOnce(() -> handMotor.set(speed));
	}

	public CommandBase driveToVelocityCommand(double eSpeed, double wSpeed) {
		return this.run(() -> drive(eSpeed, wSpeed));
	}

	public CommandBase driveToPositionCommand(double ePos, double wPos) {
		return this.runOnce(() -> driveToPosition(ePos, wPos));
	}
}

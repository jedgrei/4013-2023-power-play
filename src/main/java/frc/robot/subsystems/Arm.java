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
	CANSparkMax wrist0Motor, wrist1Motor, handMotor;
	AbsoluteEncoder elbowEncoder;
	RelativeEncoder wristEncoder;

    // pid controllers + feedforward
    private final ProfiledPIDController elbowPid = new ProfiledPIDController(
		kpElbow, kiElbow, kdElbow,
		new TrapezoidProfile.Constraints(
			maxAngularVelocityElbow, maxAngularAccelerationElbow
		)
	);
    private final ProfiledPIDController wristPid = new ProfiledPIDController(
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
        wrist0Motor = new CANSparkMax(wrist0Port, MotorType.kBrushless);
        wrist1Motor = new CANSparkMax(wrist1Port, MotorType.kBrushless);
		handMotor = new CANSparkMax(handPort, MotorType.kBrushless);
	
		// elbow0Motor.setInverted(true);
		// elbow2Motor.setInverted(true);
		// wrist1Motor.setInverted(true);

        elbowEncoder = elbow0Motor.getAbsoluteEncoder(Type.kDutyCycle);
		wristEncoder = wrist0Motor.getEncoder();
        // wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // elbowEncoder.setPositionConversionFactor(1/35.5);
        // wristEncoder.setPositionConversionFactor(1/35.5);
        // wristEncoder.setPositionConversionFactor(1/35.5);
    }

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		
	}

	// ------------------------------- METHODS ------------------------------- //
    public void drive(double eSpeed, double wSpeed) {
        // double elbowOutput = elbowPid.calculate(elbowEncoder.getVelocity(), eSpeed);
		// double elbowFf = elbowFeedforward.calculate(elbowEncoder.getVelocity(), eSpeed);
        // double wristOutput = wristPid.calculate(wristEncoder.getVelocity(), wSpeed);
		// double wristFf = wristFeedforward.calculate(wristEncoder.getVelocity(), wSpeed);
    
		// elbowLeftMotor.setVoltage(elbowOutput + elbowFf);
		// elbowRightMotor.setVoltage(elbowOutput + elbowFf);
		// wristMotor.setVoltage(wristOutput + wristFf);
        double elbowOutput = eSpeed * 3;
        // double wristOutput = wSpeed * 3;
		double wristOutput;
		if(wSpeed != 0) wristOutput = 5/6;
		else wristOutput = 0;
        elbow0Motor.set(-elbowOutput);
        elbow1Motor.set(elbowOutput);
        elbow2Motor.set(elbowOutput);
        elbow3Motor.set(-elbowOutput);
        wrist0Motor.set(wristOutput * 6 / 5);
        wrist1Motor.set(-wristOutput);
        SmartDashboard.putNumber("elbow", elbowOutput);
        SmartDashboard.putNumber("wrist", wristOutput);
	}

	public void driveToPosition(double ePosition, double wPosition) {
        double elbowOutput = elbowPid.calculate(elbowEncoder.getPosition(), ePosition);
		double elbowFf = elbowFeedforward.calculate(elbowEncoder.getPosition(), ePosition);
        double wristOutput = wristPid.calculate(wristEncoder.getPosition(), wPosition);
		double wristFf = wristFeedforward.calculate(wristEncoder.getPosition(), wPosition);

        elbow0Motor.set(elbowOutput + elbowFf);
        elbow1Motor.set(elbowOutput + elbowFf);
        elbow2Motor.set(elbowOutput + elbowFf);
        elbow3Motor.set(elbowOutput + elbowFf);
        wrist0Motor.set(wristOutput + wristFf);
        wrist1Motor.set(wristOutput + wristFf);
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

	public CommandBase driveArmCommand(double eSpeed, double wSpeed) {
		return this.run(() -> drive(eSpeed, wSpeed));
	}
}

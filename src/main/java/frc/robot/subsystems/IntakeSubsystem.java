// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSubsystem extends SubsystemBase {
	public class Arm {
		CANSparkMax armL1, armL2, armR1, armR2, elbowL, elbowR;
		RelativeEncoder armEncoder, elbowEncoder;

		public Arm(int armL1Port, int armL2Port, int armR1Port, int armR2Port, int elbowLPort, int elbowRPort) {
			armL1 = new CANSparkMax(armL1Port, MotorType.kBrushed);
			armL2 = new CANSparkMax(armL2Port, MotorType.kBrushed);
			armR1 = new CANSparkMax(armR1Port, MotorType.kBrushed);
			armR2 = new CANSparkMax(armR2Port, MotorType.kBrushed);

			armL2.setInverted(true);
			armR1.setInverted(true);

			elbowL = new CANSparkMax(elbowRPort, MotorType.kBrushless);
			elbowR = new CANSparkMax(elbowRPort, MotorType.kBrushless);

			// arm ports:   16,9,3,2
			// elbow ports: 6,5

			armEncoder = armL1.getEncoder();
			elbowEncoder = elbowL.getEncoder();

			armEncoder.setPositionConversionFactor(1 / 35.5);
			elbowEncoder.setPositionConversionFactor(1 / 2.5);
		}

		public void rotate(double speed) {
			armL1.set(speed);
			armL2.set(speed);
			armR1.set(speed);
			armR2.set(speed);
		}

		public void stop() {
			armL1.set(0);
			armL2.set(0);
			armR1.set(0);
			armR2.set(0);
		}
	}

	public class Claw {
		CANSparkMax clawMotor;

		public Claw(int clawMotorPort) {
			clawMotor = new CANSparkMax(clawMotorPort, MotorType.kBrushless);
		}

		public void grab() {
			clawMotor.set(0.5);
		}

		public void shoot() {
			clawMotor.set(-1);
			new WaitCommand(2);
		}

		public void release() {
			clawMotor.set(-0.15);
			new WaitCommand(2);
		}
		
		public void stop() {
			clawMotor.set(0);
		}
	}

	
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
    // public static final int elbowEncoderPort = 0;

	public static final int wrist0Port = 6;
	public static final int wrist1Port = 5;
    // public static final int wristEncoderPort = 0;

    public static final int handPort = 0;

    // contoller constants TODO: tune nums
    private static final double kpElbow = 1, kiElbow = 0, kdElbow = 0;
	private static final double kpWrist = 1, kiWrist = 0, kdWrist = 0;
	private static final double ksElbow = 1, kgElbow = 0, kvElbow = 0.5;
	private static final double ksWrist = 1, kgWrist = 0, kvWrist = 0.5;

	// ------------------------------- MEMBERS ------------------------------- //

	public Arm arm;
	public Claw claw;

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
	public IntakeSubsystem() {
        arm = new Arm(elbow0Port, elbow1Port, elbow2Port, elbow3Port, wrist0Port, wrist1Port);
		claw = new Claw(handPort);
    }

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		
	}
}

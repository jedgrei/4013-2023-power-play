// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Arm extends SubsystemBase {
// 	// ------------------------------ CONSTANTS ------------------------------ //
// 	// speeds
// 	public static final double maxAngularVelocityElbow = Math.PI;
// 	public static final double maxAngularVelocityWrist = Math.PI;
// 	public static final double maxAngularAccelerationElbow = Math.PI;
// 	public static final double maxAngularAccelerationWrist = Math.PI;

// 	// motor/encoder ports
//     // TODO: find arm ports
// 	public static final int elbowLeftPort = 0;
// 	public static final int elbowRightPort = 0;
//     public static final int elbowEncoderPort = 0;

// 	public static final int wristPort = 0;
//     public static final int wristEncoderPort = 0;

//     // contoller constants TODO: tune nums
//     private static final double kpElbow = 1, kiElbow = 0, kdElbow = 0;
// 	private static final double kpWrist = 1, kiWrist = 0, kdWrist = 0;
// 	private static final double ksElbow = 1, kgElbow = 0, kvElbow = 0.5;
// 	private static final double ksWrist = 1, kgWrist = 0, kvWrist = 0.5;

// 	// ------------------------------- MEMBERS ------------------------------- //
// 	// motors + encoders
//     CANSparkMax elbowLeftMotor, elbowRightMotor, wristMotor;
// 	DutyCycleEncoder elbowEncoder, wristEncoder;

//     // pid controllers + feedforward
//     private final ProfiledPIDController elbowPid = new ProfiledPIDController(
// 		kpElbow, kiElbow, kdElbow,
// 		new TrapezoidProfile.Constraints(
// 			maxAngularVelocityElbow, maxAngularAccelerationElbow
// 		)
// 	);
//     private final ProfiledPIDController wristPid = new ProfiledPIDController(
// 		kpWrist, kiWrist, kdWrist,
// 		new TrapezoidProfile.Constraints(
// 			maxAngularVelocityWrist, maxAngularAccelerationWrist
// 		)
// 	);

// 	private final ArmFeedforward elbowFeedforward = new ArmFeedforward(ksElbow, kgElbow, kvElbow);
// 	private final ArmFeedforward wristFeedforward = new ArmFeedforward(ksWrist, kgWrist, kvWrist);

// 	// ----------------------------- CONSTRUCTOR ----------------------------- //
// 	public Arm() {
//         elbowLeftMotor = new CANSparkMax(elbowLeftPort, MotorType.kBrushless);
//         elbowRightMotor = new CANSparkMax(elbowRightPort, MotorType.kBrushless);
//         wristMotor = new CANSparkMax(wristPort, MotorType.kBrushless);
	
//         elbowEncoder = new DutyCycleEncoder(elbowEncoderPort);
//         wristEncoder = new DutyCycleEncoder(wristEncoderPort);

//         elbowEncoder.setDistancePerRotation(2 * Math.PI);
//         elbowEncoder.setDistancePerRotation(2 * Math.PI);
//     }

// 	// ------------------------------- PERIODIC ------------------------------ //
// 	@Override
// 	public void periodic() {
		
// 	}

// 	// ------------------------------- METHODS ------------------------------- //
//     public void driveElbow() {

//     }

//     public void driveWrist() {

//     }
// }

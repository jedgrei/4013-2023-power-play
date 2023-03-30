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

public class Hand extends SubsystemBase {
	// ------------------------------ CONSTANTS ------------------------------ //
	// speeds and limits
	public static final double intakeSpeed = 1.0;
	public static final double outputSpeed = -1.0;
	public static final double holdingSpeed = 1.0;
	public static final int elbowCurrentLimit = 25;

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
	CANSparkMax handMotor;
	boolean isHolding = false;
	
	// ----------------------------- CONSTRUCTOR ----------------------------- //
	public Hand() {

    }

	// ------------------------------- PERIODIC ------------------------------ //
	@Override
	public void periodic() {
		
	}

	// ------------------------------- COMMANDS ------------------------------ //
	public CommandBase startIntake() {
		return this.runOnce(() -> handMotor.set(intakeSpeed));
	}

	public CommandBase stopIntake() {
		return this.runOnce(() -> handMotor.set(holdingSpeed));
	}

	public CommandBase startOutput() {
		return this.runOnce(() -> handMotor.set(outputSpeed));
	}

	public CommandBase stopHand() {
		return this.runOnce(() -> handMotor.set(intakeSpeed));
	}
}

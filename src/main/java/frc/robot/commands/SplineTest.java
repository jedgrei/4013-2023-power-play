// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmAlwaysPid;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Hand;

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SplineTest extends SequentialCommandGroup {
  /** Creates a new SplineTest. */
  public SplineTest(DriveBase drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    // Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(drive.ksVolts, drive.kvVoltSecondsPerMeter, drive.kaVoltSecondsSquaredPerMeter),
      drive.kinematics,
      10
    );
	
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(drive.kMaxSpeedMetersPerSecond, drive.kMaxAccelerationMetersPerSecondSquared).setKinematics(drive.kinematics).addConstraint(autoVoltageConstraint);
	
		// An example trajectory to follow.  All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			// Pass config
			config
		);
	
		RamseteCommand ramseteCommand = new RamseteCommand(
			exampleTrajectory,
			drive::getPose,
			new RamseteController(drive.kRamseteB, drive.kRamseteZeta),
			new SimpleMotorFeedforward(drive.ksVolts, drive.kvVoltSecondsPerMeter, drive.kaVoltSecondsSquaredPerMeter),
			drive.kinematics,
			drive::getWheelSpeeds,
			new PIDController(drive.kPDriveVel, 0, 0),
			new PIDController(drive.kPDriveVel, 0, 0),
			drive::tankDriveVolts,
			drive
		);
	
		// Reset odometry to the starting pose of the trajectory.
		drive.resetOdometry(exampleTrajectory.getInitialPose());
	
		// Run path following command, then stop at the end.
    addCommands(ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0)));
  }
}

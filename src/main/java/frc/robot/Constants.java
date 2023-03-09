// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	// constants for 
	public static class PortConstants {
		public static final int flPower = 3;
		public static final int flSpin = 7;
		public static final boolean flPowerReversed = false;
		public static final boolean flSpinReversed = false;
		public static final double flAbsoluteOffset = 0;

		public static final int frPower = 0;
		public static final int frSpin = 5;
		public static final boolean frPowerReversed = false;
		public static final boolean frSpinReversed = false;
		public static final double frAbsoluteOffset = 0;

		public static final int rrPower = 15;
		public static final int rrSpin = 11;
		public static final boolean rrPowerReversed = false;
		public static final boolean rrSpinReversed = false;
		public static final double rrAbsoluteOffset = 0;

		public static final int rlPower = 12;
		public static final int rlSpin = 8;
		public static final boolean rlPowerReversed = false;
		public static final boolean rlSpinReversed = false;
		public static final double rlAbsoluteOffset = 0;
	}

	// other constants for individual wheels
	public static class WheelConstants {
		// pid coefficients for spin motors
		public static final double kPSpin = 0.5;
		public static final double kISpin = 0;
		public static final double kDSpin = 0;

		// physical attributes of the motors
		public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
		public static final double kPowerGearRatio = 1 / 1; //TODO: find
		public static final double kSpinGearRatio = 1 / 1;
		public static final double kPowerEncoderRot2Meter = kPowerGearRatio * Math.PI * kWheelDiameterMeters;
		public static final double kSpinEncoderRot2Rad = kSpinGearRatio * 2 * Math.PI;
		public static final double kPowerEncoderRPM2MeterPerSec = kPowerEncoderRot2Meter / 60;
		public static final double kSpinEncoderRPM2RadPerSec = kSpinEncoderRot2Rad / 60;
	}

	// other constants for drive train
	public static class DriveConstants {
		// physical distances on robot
		public static final double kWheelBase = Units.inchesToMeters(22.5); // right to left wheels
		public static final double kTrackWidth = Units.inchesToMeters(22.5); // front to rear wheels
		// public static final SwerveModulePosition[] kModulePositions = null;
		
		// kinematics model for swerve drive
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
			new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
			new Translation2d(kWheelBase / 2, kTrackWidth / 2),
			new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
			new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
		);

		// maximum speeds and accelerations
		public static final double kPhysicalMaxSpeedMetersPerSecond = 0;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0;
		public static final double kTeleDriveMaxSpeedMetersPerSecond = 0;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 0;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0;
	}

	// other constants for arm
	public static class ArmConstants {

	}

	// constants specific to autonomous
	public static class AutoConstants {

		public static final double kPThetaController = 0;
		public static final Constraints kThetaControllerConstraints = null;
		public static double kPXController;
		public static double kPYController;

	}

	// constants for user input
	public static class InputConstants {
		public static final double kDeadband = 0;

	}
}

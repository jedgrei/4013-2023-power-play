package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveWheel {
	// ------------------------------ CONSTANTS ------------------------------ //
	// physical constants TODO: check wheel constants for accuracy
	private static final double wheelDiam = Units.inchesToMeters(3);
    public static final int powerMotorPinionTeeth = 12;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double powerMotorReduction = (45.0 * 22) / powerMotorPinionTeeth * 15;
    private static final double powerPositionConversion = wheelDiam * Math.PI / powerMotorReduction;
    private static final double powerVelocityConversion = powerPositionConversion / 60;

    
	// speeds + accelerations
	private static final double maxAngularVelocity = Math.PI;
	private static final double maxAngularAcceleration = 2 * Math.PI;

	// controller constants TODO: tune nums
	private static final double kpPower = 0.5, kiPower = 0, kdPower = 0;
	private static final double kpSpin = 0.9, kiSpin = 0.1, kdSpin = 0.0;
	private static final double ksPower = 1, kvPower = 3;
	private static final double ksSpin = 1, kvSpin = 0.5;

	// ------------------------------- MEMBERS ------------------------------- //
	// motors + encoders
    private final CANSparkMax powerMotor, spinMotor;
    private final RelativeEncoder powerEncoder, spinEncoder;
    // private final DutyCycleEncoder spinEncoder;
    // private final double spinEncoderOffset;

	// pid controllers + feedforward
    private final PIDController powerPid = new PIDController(kpPower, kiPower, kdPower);
	private final ProfiledPIDController spinPid = new ProfiledPIDController(
		kpSpin, kiSpin, kdSpin,
		new TrapezoidProfile.Constraints(
			maxAngularVelocity, maxAngularAcceleration
		)
	);
	private final SimpleMotorFeedforward powerFeedforward = new SimpleMotorFeedforward(ksPower, kvPower);
	private final SimpleMotorFeedforward spinFeedforward = new SimpleMotorFeedforward(ksSpin, kvSpin);
    
	// ----------------------------- CONSTRUCTOR ----------------------------- //
    public SwerveWheel(int powerPort, int spinPort, int spinEncoderPort, double spinEncoderOffsetVal) {
        powerMotor = new CANSparkMax(powerPort, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinPort, MotorType.kBrushless);

        powerEncoder = powerMotor.getEncoder();
        spinEncoder = spinMotor.getEncoder();
        // spinEncoder = new DutyCycleEncoder(spinEncoderPort);
        // spinEncoderOffset = spinEncoderOffsetVal;

        powerEncoder.setPositionConversionFactor(powerPositionConversion);
        powerEncoder.setVelocityConversionFactor(powerVelocityConversion);
        // spinEncoder.setDistancePerRotation(2 * Math.PI);
        spinEncoder.setPositionConversionFactor(Math.PI / 24);
        spinEncoder.setVelocityConversionFactor(Math.PI / 24);

        spinPid.enableContinuousInput(-Math.PI, Math.PI);
    }

	// ------------------------------- GETTERS ------------------------------- //

    public SwerveModuleState getState() {
        // return new SwerveModuleState(powerEncoder.getVelocity(), new Rotation2d(spinEncoder.getDistance()));
        return new SwerveModuleState(powerEncoder.getVelocity(), new Rotation2d(spinEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        // return new SwerveModulePosition(powerEncoder.getPosition(), new Rotation2d(spinEncoder.getDistance()));
        return new SwerveModulePosition(powerEncoder.getPosition(), new Rotation2d(spinEncoder.getPosition()));
    }

	// ------------------------------- METHODS ------------------------------- //
    public void resetEncoders() {
        powerEncoder.setPosition(0);
        spinEncoder.setPosition(0);
        // spinEncoder.reset();
    }

    public void resetSpinEncoder() {
        spinEncoder.setPosition(0);
        // spinEncoder.reset();
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(spinEncoder.getPosition()));
        // state = SwerveModuleState.optimize(state, new Rotation2d(spinEncoder.getDistance()));
		final double powerOutput = powerPid.calculate(powerEncoder.getVelocity(), state.speedMetersPerSecond);
		// final double powerFf = powerFeedforward.calculate(state.speedMetersPerSecond);
		final double spinOutput = spinPid.calculate(spinEncoder.getPosition(), state.angle.getRadians());
		// final double spinOutput = spinPid.calculate(spinEncoder.getDistance(), state.angle.getRadians());
		// final double spinFf = spinFeedforward.calculate(spinPid.getSetpoint().velocity);

        // powerMotor.set(powerOutput + powerFf);
        // spinMotor.set(spinOutput + spinFf);
        powerMotor.set(powerOutput);
        spinMotor.set(spinOutput);
    }

    public void stop() {
        powerMotor.set(0);
        spinMotor.set(0);
    }
}

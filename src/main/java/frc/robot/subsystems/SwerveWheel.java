package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.WheelConstants;

public class SwerveWheel {
	// ------------------------------ CONSTANTS ------------------------------ //
	// physical constants
	private static final double wheelDiam = 3; // TODO: find 
	private static final int encoderRes = 0; // TODO: find 

	// speeds + accelerations
	private static final double maxAngularVelocity = 0; //TODO:aaa
	private static final double maxAngularAcceleration = 2 * Math.PI;

	// controller constants TODO: tune nums
	private static final double kpPower = 1, kiPower = 0, kdPower = 0;
	private static final double kpSpin = 1, kiSpin = 0, kdSpin = 0;
	private static final double ksPower = 1, kvPower = 3;
	private static final double ksSpin = 1, kvSpin = 0.5;

	// ------------------------------- MEMBERS ------------------------------- //
	// motors + encoders
    private final CANSparkMax powerMotor, spinMotor;
    private final RelativeEncoder powerEncoder, spinEncoder;
    AbsoluteEncoder absoluteEncoder;

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
    public SwerveWheel(int powerPort, int spinPort) {
        powerMotor = new CANSparkMax(powerPort, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinPort, MotorType.kBrushless);

        powerEncoder = powerMotor.getEncoder();
        spinEncoder = spinMotor.getEncoder();
        absoluteEncoder = spinMotor.getAbsoluteEncoder(Type.kDutyCycle);

		// is this necessary? 
        powerEncoder.setPositionConversionFactor(WheelConstants.kPowerEncoderRot2Meter);
        powerEncoder.setVelocityConversionFactor(WheelConstants.kPowerEncoderRPM2MeterPerSec);
        spinEncoder.setPositionConversionFactor(WheelConstants.kSpinEncoderRot2Rad);
        spinEncoder.setVelocityConversionFactor(WheelConstants.kSpinEncoderRPM2RadPerSec);


        spinPid.enableContinuousInput(-Math.PI, Math.PI);
    
    }

	// ------------------------------- GETTERS ------------------------------- //
    public double getPowerPosition() {
        return powerEncoder.getPosition();
    }

    
    public double getSpinPosition() {
        return spinEncoder.getPosition();
    }

    public double getPowerVelocity() {
        return powerEncoder.getVelocity();
    }
    
    public double getSpinVelocity() {
        return spinEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getPosition();
        angle *= 2.0 * Math.PI;
        return angle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getPowerVelocity(), new Rotation2d(getSpinPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPowerPosition(), new Rotation2d(getSpinPosition()));
    }

	// ------------------------------- METHODS ------------------------------- //
    public void resetEncoders() {
        powerEncoder.setPosition(0);
        spinEncoder.setPosition(getAbsoluteEncoderRad());
    }


    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, new Rotation2d(spinEncoder.getPosition()));

		final double powerOutput = powerPid.calculate(powerEncoder.getVelocity(), state.speedMetersPerSecond);
		final double powerFf = powerFeedforward.calculate(state.speedMetersPerSecond);
		final double spinOutput = spinPid.calculate(spinEncoder.getPosition(), state.angle.getRadians());
		final double spinFf = spinFeedforward.calculate(spinPid.getSetpoint().velocity);

        powerMotor.set(powerOutput + powerFf);
        spinMotor.set(spinOutput + spinFf);
    }

    public void stop() {
        powerMotor.set(0);
        spinMotor.set(0);
    }
}

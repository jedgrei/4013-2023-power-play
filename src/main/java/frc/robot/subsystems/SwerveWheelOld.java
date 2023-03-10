// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.WheelConstants;

// public class SwerveWheelOld {
//     CANSparkMax powerMotor, spinMotor;
//     RelativeEncoder powerEncoder, spinEncoder;
//     PIDController spinPid;

//     AbsoluteEncoder absoluteEncoder;
//     boolean absoluteEncoderReversed;
//     double absoluteEncoderOffsetRad;
    
    
//     public SwerveWheel(int powerPort, int spinPort, boolean powerReversed, boolean spinReversed, double absoluteEncoderOffset) {
//         powerMotor = new CANSparkMax(powerPort, MotorType.kBrushless);
//         spinMotor = new CANSparkMax(spinPort, MotorType.kBrushless);

//         powerMotor.setInverted(powerReversed);
//         spinMotor.setInverted(spinReversed);

//         powerEncoder = powerMotor.getEncoder();
//         spinEncoder = spinMotor.getEncoder();

//         powerEncoder.setPositionConversionFactor(WheelConstants.kPowerEncoderRot2Meter);
//         powerEncoder.setVelocityConversionFactor(WheelConstants.kPowerEncoderRPM2MeterPerSec);
//         spinEncoder.setPositionConversionFactor(WheelConstants.kSpinEncoderRot2Rad);
//         spinEncoder.setVelocityConversionFactor(WheelConstants.kSpinEncoderRPM2RadPerSec);

//         absoluteEncoder = spinMotor.getAbsoluteEncoder(Type.kDutyCycle);
//         this.absoluteEncoderReversed = spinReversed;
//         this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

//         powerEncoder.setPositionConversionFactor(WheelConstants.kPowerEncoderRot2Meter);
//         powerEncoder.setVelocityConversionFactor(WheelConstants.kPowerEncoderRPM2MeterPerSec);
//         spinEncoder.setPositionConversionFactor(WheelConstants.kSpinEncoderRot2Rad);
//         spinEncoder.setVelocityConversionFactor(WheelConstants.kSpinEncoderRPM2RadPerSec);

//         spinPid = new PIDController(WheelConstants.kPSpin, WheelConstants.kISpin, WheelConstants.kDSpin);
//         spinPid.enableContinuousInput(-Math.PI, Math.PI);
        
//         resetEncoders();
//     }

//     public double getPowerPosition() {
//         return powerEncoder.getPosition();
//     }

    
//     public double getSpinPosition() {
//         return spinEncoder.getPosition();
//     }

//     public double getPowerVelocity() {
//         return powerEncoder.getVelocity();
//     }
    
//     public double getSpinVelocity() {
//         return spinEncoder.getVelocity();
//     }

//     public double getAbsoluteEncoderRad() {
//         double angle = absoluteEncoder.getPosition();
//         angle *= 2.0 * Math.PI;
//         angle -= absoluteEncoderOffsetRad;
//         return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
//     }

//     public void resetEncoders() {
//         powerEncoder.setPosition(0);
//         spinEncoder.setPosition(getAbsoluteEncoderRad());
//     }

//     public SwerveModuleState getState() {
//         return new SwerveModuleState(getPowerVelocity(), new Rotation2d(getSpinPosition()));
//     }

//     public SwerveModulePosition getModulePosition() {
//         return new SwerveModulePosition(getPowerPosition(), new Rotation2d(getSpinPosition()));
//     }

//     public void setDesiredState(SwerveModuleState state) {
//         if(Math.abs(state.speedMetersPerSecond) < 0.001) {
//             stop();
//             return;
//         }
//         state = SwerveModuleState.optimize(state, getState().angle);
//         powerMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
//         spinMotor.set(spinPid.calculate(getSpinPosition(), state.angle.getRadians()));
//     }

//     public void stop() {
//         powerMotor.set(0);
//         spinMotor.set(0);
//     }
// }

// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.ADIS16448_IMU;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.PortConstants;

// public class SwerveDriveOld extends SubsystemBase {
//     SwerveWheel frontLeft = new SwerveWheel(
//         PortConstants.flPower,
//         PortConstants.flSpin,
//         PortConstants.flPowerReversed,
//         PortConstants.flSpinReversed,
//         PortConstants.flAbsoluteOffset
//     );
//     SwerveWheel frontRight = new SwerveWheel(
//         PortConstants.frPower,
//         PortConstants.frSpin,
//         PortConstants.frPowerReversed,
//         PortConstants.frSpinReversed,
//         PortConstants.frAbsoluteOffset
//     );
//     SwerveWheel rearRight = new SwerveWheel(
//         PortConstants.rrPower,
//         PortConstants.rrSpin,
//         PortConstants.rrPowerReversed,
//         PortConstants.rrSpinReversed,
//         PortConstants.rrAbsoluteOffset
//     );
//     SwerveWheel rearLeft = new SwerveWheel(
//         PortConstants.rlPower,
//         PortConstants.rlSpin,
//         PortConstants.rlPowerReversed,
//         PortConstants.rlSpinReversed,
//         PortConstants.rlAbsoluteOffset
//     );
//     ADIS16448_IMU gyro;
//     SwerveDriveOdometry odometer = new SwerveDriveOdometry(
//         DriveConstants.kDriveKinematics,
//         new Rotation2d(0),
//         getModulePositions()
//     );

//     public SwerveDriveOld() {
//         // what ??
//         new Thread(() -> {
//             try {
//                 Thread.sleep(1000);
//                 zeroHeading();
//             }
//             catch (Exception e) {}
//         }).start();
//     }


//     public void zeroHeading() {
//         gyro.reset();
//     }

//     public double getHeading() {
//         return Math.IEEEremainder(gyro.getAngle(), 260);
//     }

//     public Rotation2d getRotation2d() {
//         return Rotation2d.fromDegrees(getHeading());
//     }

//     public Pose2d getPose() {
//         return odometer.getPoseMeters();
//     }

//     public SwerveModulePosition[] getModulePositions() {
//         return new SwerveModulePosition[] {
//             frontLeft.getModulePosition(),
//             frontRight.getModulePosition(),
//             rearLeft.getModulePosition(),
//             rearRight.getModulePosition(),
//         };
//     }

//     public void resetOdometry(Pose2d pose) {
//         odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
//     }

//     public void periodic() {
//         odometer.update(getRotation2d(), getModulePositions());
//     }

//     public void stopModules() {
//         frontLeft.stop();
//         frontRight.stop();
//         rearLeft.stop();
//         rearRight.stop();
//     }

//     public void setModuleStates(SwerveModuleState[] desiredStates) {
//         SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
//         frontLeft.setDesiredState(desiredStates[0]);
//         frontRight.setDesiredState(desiredStates[1]);
//         rearLeft.setDesiredState(desiredStates[2]);
//         rearRight.setDesiredState(desiredStates[3]);
    
//     }
// }

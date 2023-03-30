package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Hand;

public class ShortDrive extends SequentialCommandGroup {
    public ShortDrive(DriveBase drive) {
        addCommands(
            drive.driveForward(0.5),
            new WaitCommand(3),
            drive.stop()
        );
    }
}

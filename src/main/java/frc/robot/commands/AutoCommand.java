package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Hand;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(DriveBase drive, Arm arm, Hand hand) {
    }
}

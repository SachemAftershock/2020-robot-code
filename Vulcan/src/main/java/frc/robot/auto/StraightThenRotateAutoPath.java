package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LinearDriveCommand;
import frc.robot.commands.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class StraightThenRotateAutoPath extends SequentialCommandGroup {

    public StraightThenRotateAutoPath() {
        addCommands(
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30),

            new RotateDriveCommand(DriveSubsystem.getInstance(), 45)
        );
    }
}
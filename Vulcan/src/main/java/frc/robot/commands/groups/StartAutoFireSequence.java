package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.shooter.StartFeederCommand;
import frc.robot.commands.superstructure.storage.StartStorageBeltCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartAutoFireSequence extends ParallelCommandGroup {

    public StartAutoFireSequence(ShooterSubsystem shooter, StorageSubsystem storage) {
        addCommands(
            new StartFeederCommand(shooter),
            new StartStorageBeltCommand(storage)
        );
    }
}
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.IngestIntakeCommand;
import frc.robot.commands.superstructure.storage.StartStorageBeltCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class ManualIngestCommand extends ParallelCommandGroup {

    public ManualIngestCommand(IntakeSubsystem intake, StorageSubsystem storage) {
        addCommands(
            new IngestIntakeCommand(intake),
            new StartStorageBeltCommand(storage)
        );
    }
}
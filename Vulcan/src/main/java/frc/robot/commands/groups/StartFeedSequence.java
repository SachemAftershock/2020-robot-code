package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.DeployIntakeCommand;
import frc.robot.commands.superstructure.intake.IngestIntakeCommand;
import frc.robot.commands.superstructure.storage.OpenChamberValveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartFeedSequence extends ParallelCommandGroup {

    public StartFeedSequence(IntakeSubsystem intake, StorageSubsystem storage) {
        addCommands(
            new IngestIntakeCommand(intake),
            new DeployIntakeCommand(intake),
            new OpenChamberValveCommand(storage)
        );
    }
}
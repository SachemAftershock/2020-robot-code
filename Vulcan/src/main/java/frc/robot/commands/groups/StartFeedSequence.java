package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.intake.DeployIntakeCommand;
import frc.robot.commands.superstructure.intake.IngestIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartFeedSequence extends SequentialCommandGroup {

    public StartFeedSequence(IntakeSubsystem intake, StorageSubsystem storage) {
        addCommands(
            new IngestIntakeCommand(intake),
            new DeployIntakeCommand(intake)
        );
    }
}
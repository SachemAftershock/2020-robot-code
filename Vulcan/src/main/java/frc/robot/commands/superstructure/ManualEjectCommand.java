package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.EjectIntakeCommand;
import frc.robot.commands.superstructure.storage.ReverseStorageBeltCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class ManualEjectCommand extends ParallelCommandGroup {

    public ManualEjectCommand(IntakeSubsystem intake, StorageSubsystem storage) {
        addCommands(
            new EjectIntakeCommand(intake),
            new ReverseStorageBeltCommand(storage)
        );
    }
}
package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.StopIntakeCommand;
import frc.robot.commands.superstructure.storage.StopStorageBeltCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StopManualDriveCommand extends ParallelCommandGroup {

    public StopManualDriveCommand(IntakeSubsystem intake, StorageSubsystem storage) {
        addCommands(
            new StopIntakeCommand(intake),
            new StopStorageBeltCommand(storage)
        );
    }
}
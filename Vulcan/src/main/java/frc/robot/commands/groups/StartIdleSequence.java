package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.shooter.StopShooterCommand;
import frc.robot.commands.superstructure.storage.StopStorageBeltCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartIdleSequence extends ParallelCommandGroup{

    public StartIdleSequence(IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {
        addCommands(
            new RetractIntakeCommand(intake),
            new StopShooterCommand(shooter),
            new StopStorageBeltCommand(storage)
        );
    }
}
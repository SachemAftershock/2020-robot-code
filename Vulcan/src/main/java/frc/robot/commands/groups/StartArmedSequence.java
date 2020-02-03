package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.superstructure.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.intake.StopIntakeCommand;
import frc.robot.commands.superstructure.shooter.ReachCalculatedTargetRPM;
import frc.robot.commands.superstructure.storage.LoadFrontMagazineCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartArmedSequence extends ParallelCommandGroup {

    public StartArmedSequence(IntakeSubsystem intake, ShooterSubsystem shooter, StorageSubsystem storage) {
        addCommands(
            new LoadFrontMagazineCommand(storage),
            new StopIntakeCommand(intake),
            new RetractIntakeCommand(intake),
            new ReachCalculatedTargetRPM(shooter)
        );
    }
}
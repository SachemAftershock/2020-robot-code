package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.intake.RetractIntakeCommand;
import frc.robot.commands.superstructure.intake.StopIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class StartArmedSequence extends SequentialCommandGroup {

    public StartArmedSequence(IntakeSubsystem intake, ShooterSubsystem shooter, StorageSubsystem storage) {
        addCommands(
            new StopIntakeCommand(intake),
            new RetractIntakeCommand(intake)
        );
    }
}
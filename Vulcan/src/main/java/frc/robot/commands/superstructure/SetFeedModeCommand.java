package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

public class SetFeedModeCommand extends InstantCommand {

    private final SuperstructureSubsystem mSuperstructure;

    public SetFeedModeCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {
        mSuperstructure.setMode(SuperstructureMode.eFeed);
    }
}

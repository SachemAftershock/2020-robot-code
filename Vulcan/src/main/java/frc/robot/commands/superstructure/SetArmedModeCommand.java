package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

public class SetArmedModeCommand extends CommandBase {

    private final SuperstructureSubsystem mSuperstructure;

    public SetArmedModeCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {
        mSuperstructure.setMode(SuperstructureMode.eArmed);
    }
}

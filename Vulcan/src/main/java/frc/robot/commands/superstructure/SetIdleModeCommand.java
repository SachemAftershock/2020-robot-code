package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

public class SetIdleModeCommand extends InstantCommand {

    private final SuperstructureSubsystem mSuperstructure;

    public SetIdleModeCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {
        if(mSuperstructure.getCurrentMode() != SuperstructureMode.eArmed) {
            mSuperstructure.setMode(SuperstructureMode.eIdle);
        }
    }
}

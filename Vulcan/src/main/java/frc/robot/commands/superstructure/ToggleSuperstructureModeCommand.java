package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

public class ToggleSuperstructureModeCommand extends CommandBase {

    private final SuperstructureSubsystem mSuperstructure;
    private boolean mIsFinished;

    public ToggleSuperstructureModeCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        if(mSuperstructure.getCurrentMode() != SuperstructureMode.eFeed) {
            mSuperstructure.setMode(SuperstructureMode.eFeed);
        } else {
            mSuperstructure.setMode(SuperstructureMode.eIdle);
        }
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

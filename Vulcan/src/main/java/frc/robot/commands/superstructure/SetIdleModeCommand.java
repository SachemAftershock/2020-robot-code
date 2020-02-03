package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

public class SetIdleModeCommand extends CommandBase {

    private final SuperstructureSubsystem mSuperstructure;
    private boolean mIsFinished;

    public SetIdleModeCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mSuperstructure.setMode(SuperstructureMode.eIdle);
        mIsFinished = false;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

package frc.robot.commands.superstructure.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class ReverseStorageBeltCommand extends CommandBase {
    
    private final StorageSubsystem mStorage;
    private boolean mIsFinished;

    public ReverseStorageBeltCommand(StorageSubsystem storage) {
        mStorage = storage;
        addRequirements(mStorage);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mStorage.reverseBelt();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}
package frc.robot.commands.superstructure.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class CloseChamberValveCommand extends CommandBase {

    private final StorageSubsystem mStorage;
    private boolean mIsFinished;

    public CloseChamberValveCommand(StorageSubsystem storage) {
        mStorage = storage;
        addRequirements(mStorage);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mStorage.closeChamberValve();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
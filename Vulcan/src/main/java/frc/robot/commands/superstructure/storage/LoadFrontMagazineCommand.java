package frc.robot.commands.superstructure.storage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class LoadFrontMagazineCommand extends CommandBase {

    private final StorageSubsystem mStorage;

    public LoadFrontMagazineCommand(StorageSubsystem storage) {
        mStorage = storage;
        addRequirements(mStorage);
    }

    @Override
    public void execute() {
        mStorage.runBelt();
    }

    @Override
    public void end(boolean interrupted) {
        mStorage.closeChamberValve();
        mStorage.stopBelt();
    }

    @Override
    public boolean isFinished() {
        return mStorage.isChamberLoaded();
    }
}
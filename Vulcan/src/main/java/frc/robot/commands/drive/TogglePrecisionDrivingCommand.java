package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TogglePrecisionDrivingCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private boolean mIsFinished;

    public TogglePrecisionDrivingCommand(DriveSubsystem drive) {
        mDrive = drive;
        addRequirements(mDrive);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mDrive.togglePrecisionDriving();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

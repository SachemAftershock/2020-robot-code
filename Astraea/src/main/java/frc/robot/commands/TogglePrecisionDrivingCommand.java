package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class TogglePrecisionDrivingCommand extends CommandBase {

    private DrivebaseSubsystem mDrivebaseSubsystem;
    private boolean completed;

    public TogglePrecisionDrivingCommand(DrivebaseSubsystem theDrivebaseSubsystem) {
        mDrivebaseSubsystem = theDrivebaseSubsystem;
        addRequirements(mDrivebaseSubsystem);
        completed = false;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mDrivebaseSubsystem.togglePrecisionDriving();
        completed = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return completed;
    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class ManualDriveCommand extends CommandBase {
 
    DrivebaseSubsystem mDrivebaseSubsystem;
    XboxController mXboxController;

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mDrivebaseSubsystem.manualDrive(mXboxController);
    }

    public ManualDriveCommand(DrivebaseSubsystem theDrivebaseSubsystem, XboxController theXboxController) {
        mDrivebaseSubsystem = theDrivebaseSubsystem;
        mXboxController = theXboxController;
        addRequirements(mDrivebaseSubsystem);
    }

}

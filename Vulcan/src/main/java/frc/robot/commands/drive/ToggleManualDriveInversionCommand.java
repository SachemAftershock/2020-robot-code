package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleManualDriveInversionCommand extends InstantCommand {

    private DriveSubsystem mDrive;

    public ToggleManualDriveInversionCommand(DriveSubsystem drive) {
        mDrive = drive;
        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        if(mDrive.isManualDriveInverted()) {
            mDrive.revertManualDrive();
        } else {
            mDrive.invertManualDrive();
        }
    }
}

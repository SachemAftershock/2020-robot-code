package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumble;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ToggleDrivebaseGearingCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private XboxController mController;
    private boolean mIsFinished;

    public ToggleDrivebaseGearingCommand(DriveSubsystem drive, XboxController controller) {
        mDrive = drive;
        mController = controller;
        addRequirements(mDrive);
        mIsFinished = false;
    }

    @Override
    public void execute() {
        mDrive.toggleDrivebaseGearing();
        (new ControllerRumble(mController, 1, 0.25)).start();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

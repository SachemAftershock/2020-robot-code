package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {
 
    private DriveSubsystem mDrive;
    private XboxController mController;

    public ManualDriveCommand(DriveSubsystem drive, XboxController controller) {
        mDrive = drive;
        mController = controller;
        addRequirements(mDrive);
    }
    
    @Override
    public void execute() {
        mDrive.manualDrive(Util.deadband(mController.getY(Hand.kLeft), Constants.kDeadbandTolerance), 
                        Util.deadband(mController.getX(Hand.kRight), Constants.kDeadbandTolerance));
    }
}

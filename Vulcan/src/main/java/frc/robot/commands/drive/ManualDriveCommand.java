package frc.robot.commands.drive;

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
        final double pow = Util.deadband(mController.getY(Hand.kLeft), Constants.kJoystickDeadbandTolerance);
        final double rot = Util.deadband(mController.getX(Hand.kRight), Constants.kJoystickDeadbandTolerance);
        final boolean leftTriggerPressed = mController.getTriggerAxis(Hand.kLeft) >= 0.5;
        final boolean rightTriggerPressed = mController.getTriggerAxis(Hand.kRight) >= 0.5;
        final boolean wantDeccelerate = leftTriggerPressed || rightTriggerPressed;
        mDrive.manualDrive(pow, rot, wantDeccelerate);
    }
}

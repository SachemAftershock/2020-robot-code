package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ManualDriveCommand extends CommandBase {
 
    private DriveSubsystem mDrive;
    private XboxController mController;
    private double prevPow, prevRot;

    public ManualDriveCommand(DriveSubsystem drive, XboxController controller) {
        mDrive = drive;
        mController = controller;
        addRequirements(mDrive);

        prevPow = 0;
        prevRot = 0;
    }
    
    @Override
    public void execute() {
        double pow = Util.deadband(mController.getY(Hand.kLeft), Constants.kJoystickDeadbandTolerance);
        double rot = Util.deadband(mController.getX(Hand.kRight), Constants.kJoystickDeadbandTolerance);
        final boolean leftTriggerPressed = mController.getTriggerAxis(Hand.kLeft) >= 0.5;
        final boolean rightTriggerPressed = mController.getTriggerAxis(Hand.kRight) >= 0.5;
        if(leftTriggerPressed || rightTriggerPressed) { 
            pow = prevPow * DriveConstants.kThrottleDecelerationProportion;
            rot = prevRot * DriveConstants.kRotationalDecelerationProportion;
        }
        prevPow = pow;
        prevRot = rot;
        mDrive.manualDrive(pow, rot);
    }
}

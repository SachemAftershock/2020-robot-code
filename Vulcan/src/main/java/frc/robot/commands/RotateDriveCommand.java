package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mThetaSetpoint;
    
    //Field Relative Rotation, downfield is 0deg
    public RotateDriveCommand(DriveSubsystem drive, double thetaSetpoint) {
        mDrive = drive;
        mThetaSetpoint = thetaSetpoint;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.startAutoRotate(mThetaSetpoint);
    }

    @Override
    public void execute() {
        mDrive.runAutoRotate();
    }

    @Override
    public boolean isFinished() {
        return mDrive.rotateTargetReached();
    }
}
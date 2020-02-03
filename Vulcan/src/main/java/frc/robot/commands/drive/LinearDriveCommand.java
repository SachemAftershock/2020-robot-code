package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class LinearDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mSetpoint;

    public LinearDriveCommand(DriveSubsystem drive, double setpoint) {
        mDrive = drive;
        mSetpoint = setpoint;
        addRequirements(mDrive);
    }   

    @Override
    public void initialize() {
        mDrive.startAutoDrive(mSetpoint);
    }

    @Override
    public void execute() {
        mDrive.runAutoDrive();
    }

    @Override
    public boolean isFinished() {
        return mDrive.linearDriveTargetReached();
    }
}
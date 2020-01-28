package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ToggleShooterElevationCommand extends CommandBase {

    private ShooterSubsystem mShooterSubsystem;
    private XboxController mXboxController;
    private boolean mIsFinished;

    public ToggleShooterElevationCommand(ShooterSubsystem theShooterSubsystem, XboxController theXboxController) {
        mShooterSubsystem = theShooterSubsystem;
        mXboxController = theXboxController;
        addRequirements(mShooterSubsystem);
        mIsFinished = false;
    }


    @Override
    public void execute() {
        mShooterSubsystem.toggleElevation();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

    @Override
    public void end(boolean interrupted) {
    }

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ToggleShooterElevationCommand extends CommandBase {

    private ShooterSubsystem mShooterSubsystem;
    private XboxController mXboxController;
    private boolean completed;

    public ToggleShooterElevationCommand(ShooterSubsystem theShooterSubsystem, XboxController theXboxController) {
        mShooterSubsystem = theShooterSubsystem;
        mXboxController = theXboxController;
        addRequirements(mShooterSubsystem);
        completed = false;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mShooterSubsystem.toggleElevation();
        mXboxController.setRumble(RumbleType.kLeftRumble, 1.0);
        mXboxController.setRumble(RumbleType.kRightRumble, 1.0);
        completed = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return completed;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

}

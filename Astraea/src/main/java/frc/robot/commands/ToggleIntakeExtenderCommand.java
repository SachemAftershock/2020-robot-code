package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ToggleIntakeExtenderCommand extends CommandBase {

    private IntakeSubsystem mIntake;
    private boolean completed;

    public ToggleIntakeExtenderCommand(IntakeSubsystem intakeSubsystem) {
        mIntake = intakeSubsystem;
        addRequirements(mIntake);
        completed = false;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mIntake.toggleIntakeExtender();
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimbShaftUpwardCommand extends CommandBase {

    boolean completed; 
    ClimberSubsystem mClimberSubsystem;

    public ExtendClimbShaftUpwardCommand(ClimberSubsystem theClimberSubsystem) {
        mClimberSubsystem = theClimberSubsystem;
        addRequirements(mClimberSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        mClimberSubsystem.setClimberShaftSpeed(0.0);
        completed = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        completed = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return completed;
//        return isTimedOut();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mClimberSubsystem.setClimberShaftSpeed(0.0);
    }

}

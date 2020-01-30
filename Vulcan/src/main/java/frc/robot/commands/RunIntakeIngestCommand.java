package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeIngestCommand extends CommandBase  {
    
    private IntakeSubsystem mIntake;

    public RunIntakeIngestCommand(IntakeSubsystem intake) {
        mIntake = intake;
    }

    @Override
    public void initialize() {
        mIntake.startIntakeMotor();
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stopIntakeMotor();
    }
}
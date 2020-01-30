package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeEjectCommand extends CommandBase  {
    
    private IntakeSubsystem mIntake;

    public RunIntakeEjectCommand(IntakeSubsystem intake) {
        mIntake = intake;
    }

    @Override
    public void initialize() {
        mIntake.ejectIntakeMotor();
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.stopIntakeMotor();
    }
}
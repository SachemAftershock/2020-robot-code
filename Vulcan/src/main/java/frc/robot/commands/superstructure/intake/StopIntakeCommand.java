package frc.robot.commands.superstructure.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase {

    private final IntakeSubsystem mIntake;
    private boolean mIsFinished;

    public StopIntakeCommand(IntakeSubsystem intake) {
        mIntake = intake;
        addRequirements(mIntake);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mIntake.stopIntakeMotor();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}
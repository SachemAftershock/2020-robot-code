package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeExtenderCommand extends CommandBase {

    private IntakeSubsystem mIntake;
    private boolean mIsFinished;

    public ToggleIntakeExtenderCommand(IntakeSubsystem intakeSubsystem) {
        mIntake = intakeSubsystem;
        addRequirements(mIntake);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mIntake.toggleIntakeExtender();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

package frc.robot.commands.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StartFeederCommand extends CommandBase {

    private final ShooterSubsystem mShooter;
    private boolean mIsFinished;

    public StartFeederCommand(ShooterSubsystem shooter) {
        mShooter = shooter;
        addRequirements(mShooter);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mShooter.startFeeder();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
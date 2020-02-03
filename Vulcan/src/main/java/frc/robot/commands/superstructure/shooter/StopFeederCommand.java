package frc.robot.commands.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopFeederCommand extends CommandBase {

    private final ShooterSubsystem mShooter;
    private boolean mIsFinished;

    public StopFeederCommand(ShooterSubsystem shooter) {
        mShooter = shooter;
        addRequirements(mShooter);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mShooter.stopFeeder();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
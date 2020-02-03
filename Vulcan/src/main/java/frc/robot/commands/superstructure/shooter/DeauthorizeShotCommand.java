package frc.robot.commands.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperstructureSubsystem;

public class DeauthorizeShotCommand extends CommandBase {

    private final SuperstructureSubsystem mSuperstructure;
    private boolean mIsFinished;

    public DeauthorizeShotCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        mSuperstructure.deauthorizeShot();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}
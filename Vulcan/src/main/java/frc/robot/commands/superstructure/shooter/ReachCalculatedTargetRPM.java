package frc.robot.commands.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ReachCalculatedTargetRPM extends CommandBase {
    
    private ShooterSubsystem mShooter;

    public ReachCalculatedTargetRPM(ShooterSubsystem shooter) {
        mShooter = shooter;
        addRequirements(mShooter);
    }

    @Override
    public void execute() {
        mShooter.reachCalculatedTargetRPM();
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopShooter();
    }
}
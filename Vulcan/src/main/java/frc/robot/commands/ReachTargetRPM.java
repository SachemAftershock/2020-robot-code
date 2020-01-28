package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ReachTargetRPM extends CommandBase {
    private ShooterSubsystem mShooter;

    public ReachTargetRPM(ShooterSubsystem shooter) {
        mShooter = shooter;
    }

    @Override
    public void execute() {
        mShooter.reachTargetRPM();
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.stopShooter();
    }
}
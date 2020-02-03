package frc.robot.commands.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperstructureSubsystem;

public class AuthorizeShotCommand extends CommandBase {

    private final SuperstructureSubsystem mSuperstructure;

    public AuthorizeShotCommand(SuperstructureSubsystem superstructure) {
        mSuperstructure = superstructure;
        addRequirements(mSuperstructure);
    }

    @Override
    public void initialize() {
        mSuperstructure.authorizeShot();
    }
}
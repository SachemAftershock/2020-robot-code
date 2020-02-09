package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseElevatorLevelCommand extends CommandBase {

    private final ClimberSubsystem mClimber;

    public RaiseElevatorLevelCommand(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void execute() {
        mClimber.raiseElevatorLevel();
    }

    @Override
    public boolean isFinished() {
        return mClimber.isAtDesiredPosition();
    }
}
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class LowerElevatorLevelCommand extends CommandBase {

    private final ClimberSubsystem mClimber;

    public LowerElevatorLevelCommand(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void execute() {
        mClimber.lowerElevatorLevel();
    }

    @Override
    public boolean isFinished() {
        return mClimber.isAtDesiredPosition();
    }
}
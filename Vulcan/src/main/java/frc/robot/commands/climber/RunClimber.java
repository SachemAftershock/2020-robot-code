package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimber extends CommandBase {

    private final ClimberSubsystem mClimber;

    public RunClimber(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void execute() {
        mClimber.driveLifter();
    }

    @Override
    public void end(boolean interrupted) {
        mClimber.stopLifter();
    }
}
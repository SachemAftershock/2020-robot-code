package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelControllerSubsystem;

public class ToggleWheelExtenderCommand extends CommandBase {

    private WheelControllerSubsystem mWheelController;
    private boolean completed;

    public ToggleWheelExtenderCommand(WheelControllerSubsystem wheelController) {
        mWheelController = wheelController;
        addRequirements(mWheelController);
        completed = false;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mWheelController.toggleExtender();
        completed = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return completed;
    }

}

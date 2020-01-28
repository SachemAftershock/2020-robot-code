package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelControllerSubsystem;

public class ToggleWheelExtenderCommand extends CommandBase {

    private WheelControllerSubsystem mWheelController;
    private boolean mIsFinished;

    public ToggleWheelExtenderCommand(WheelControllerSubsystem wheelController) {
        mWheelController = wheelController;
        addRequirements(mWheelController);
        mIsFinished = false;
    }

    @Override
    public void execute() {
        mWheelController.toggleExtender();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelControllerSubsystem;

public class WheelColorControlCommand extends CommandBase {
    private final WheelControllerSubsystem mWheelController;

    public WheelColorControlCommand(WheelControllerSubsystem subsystem) {
        mWheelController = subsystem;
        addRequirements(mWheelController);
    }

    @Override
    public void initialize() {
        mWheelController.turnToTargetColor();
    }

    @Override
    public void end(boolean interrupted) {
        mWheelController.stopColorTargeting();
    }

    @Override
    public boolean isFinished() {
        return mWheelController.isTargetColorFound();
    }
}
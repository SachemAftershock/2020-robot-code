package frc.robot.commands.wheelcontroller;

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
        mWheelController.startColorTargeting();
    }

    @Override
    public void end(boolean interrupted) {
        mWheelController.stopComand();
    }

    @Override
    public boolean isFinished() {
        return mWheelController.isTargetColorFound();
    }
}
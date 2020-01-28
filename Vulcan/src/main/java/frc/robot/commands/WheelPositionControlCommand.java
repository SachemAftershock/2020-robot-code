package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelControllerSubsystem;

public class WheelPositionControlCommand extends CommandBase {
    private final WheelControllerSubsystem mWheelController;
    private final Timer mTimer;
    private final double kTimeoutInSeconds = 30.0; //TODO: Time a good value for this

    public WheelPositionControlCommand(WheelControllerSubsystem subsystem) {
        mWheelController = subsystem;
        addRequirements(mWheelController);
        mTimer = new Timer();
    }

    @Override
    public void initialize() {
        mTimer.start();
        mWheelController.startPositionControl();
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
        mWheelController.endPositionControl();
    }

    @Override
    public boolean isFinished() {
        if(mTimer.hasPeriodPassed(kTimeoutInSeconds)) {
            System.out.println("ERROR: WHEEL POSITION COMMAND TIMED OUT");
            return true;
        } else if(mWheelController.isPositionConditionMet()) {
            return true;
        }
        return false;
    }
}
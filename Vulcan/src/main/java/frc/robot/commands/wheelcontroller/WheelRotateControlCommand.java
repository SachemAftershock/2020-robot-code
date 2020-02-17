package frc.robot.commands.wheelcontroller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelControllerSubsystem;

public class WheelRotateControlCommand extends CommandBase {
    private final WheelControllerSubsystem mWheelController;
    private final Timer mTimer;
    private final double kTimeoutInSeconds = 30.0; //TODO: Time a good value for this

    public WheelRotateControlCommand(WheelControllerSubsystem subsystem) {
        mWheelController = subsystem;
        addRequirements(mWheelController);
        mTimer = new Timer();
    }

    @Override
    public void initialize() {
        mTimer.start();
        mWheelController.startRotationControl();
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
        mWheelController.endRotationControl();
    }

    @Override
    public boolean isFinished() {
        if(mTimer.hasPeriodPassed(kTimeoutInSeconds)) {
            DriverStation.reportError("ERROR: WHEEL POSITION COMMAND TIMED OUT", false);
            return true;
        } else if(mWheelController.isRotateConditionMet()) {
            return true;
        }
        return false;
    }
}
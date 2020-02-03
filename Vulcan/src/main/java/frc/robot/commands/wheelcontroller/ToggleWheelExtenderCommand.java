package frc.robot.commands.wheelcontroller;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;

public class ToggleWheelExtenderCommand extends CommandBase {

    private WheelControllerSubsystem mWheelController;
    private CollisionAvoidanceSubsystem mCollisionAvoidance;
    private boolean mIsFinished;

    public ToggleWheelExtenderCommand(WheelControllerSubsystem wheelController, CollisionAvoidanceSubsystem collisionAvoidance) {
        mWheelController = wheelController;
        mCollisionAvoidance = collisionAvoidance;
        addRequirements(mWheelController);
        mIsFinished = false;
    }

    @Override
    public void execute() {
        mWheelController.toggleExtender();
        mCollisionAvoidance.setColorWheelStandoff();
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}

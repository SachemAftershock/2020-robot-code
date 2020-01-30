package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerRumble;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;

public class ToggleCollisionAvoidanceCommand extends CommandBase {
    
    private final CollisionAvoidanceSubsystem mCollisionAvoidance;
    private final XboxController mController;
    private boolean mIsFinished;

    public ToggleCollisionAvoidanceCommand(CollisionAvoidanceSubsystem collisionAvoidance, XboxController controller) {
        mCollisionAvoidance = collisionAvoidance;
        mController = controller;
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        if(mCollisionAvoidance.isCollisionAvoidanceEnabled()) {
            mCollisionAvoidance.disableCollisionAvoidance();
            (new ControllerRumble(mController, 1)).start();;
        } else {
            mCollisionAvoidance.enableCollisionAvoidance();
            (new ControllerRumble(mController, 2)).start();
        }
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }
}
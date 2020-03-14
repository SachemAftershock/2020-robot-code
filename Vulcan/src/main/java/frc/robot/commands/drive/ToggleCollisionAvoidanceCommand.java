package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.ControllerRumble;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;

/**
 * Command to Toggle whether Collision Avoidance is enabled or disabled
 * 
 * @author Shreyas Prasad
 */
public class ToggleCollisionAvoidanceCommand extends CommandBase {
    
    private final CollisionAvoidanceSubsystem mCollisionAvoidance;
    private final XboxController mController;
    private boolean mIsFinished;

    /**
     * Constructor for ToggleCollisionAvoidanceCommand Class
     * 
     * @param collisionAvoidance CollisionAvoidanceSubsystem singleton instance
     * 
     * @param controller Primary Xbox Controller for rumble
     */
    public ToggleCollisionAvoidanceCommand(CollisionAvoidanceSubsystem collisionAvoidance, XboxController controller) {
        mCollisionAvoidance = collisionAvoidance;
        mController = controller;
        mIsFinished = false;
        addRequirements(mCollisionAvoidance);
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
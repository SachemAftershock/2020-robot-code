package frc.robot.commands.wheelcontroller;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;

public class ToggleWheelExtenderCommand extends InstantCommand {

    private final WheelControllerSubsystem mWheelController;
    private final CollisionAvoidanceSubsystem mCollisionAvoidance;

    public ToggleWheelExtenderCommand(WheelControllerSubsystem wheelController, CollisionAvoidanceSubsystem collisionAvoidance) {
        mWheelController = wheelController;
        mCollisionAvoidance = collisionAvoidance;
        addRequirements(mWheelController);
        addRequirements(mCollisionAvoidance);
    }

    @Override
    public void execute() {
        if(mWheelController.isExtended()) {
            mWheelController.retractExtender();
            mCollisionAvoidance.setStandardStandoff();
        } else {
            mWheelController.deployExtender();
            mCollisionAvoidance.setColorWheelStandoff();
        }
    }
}
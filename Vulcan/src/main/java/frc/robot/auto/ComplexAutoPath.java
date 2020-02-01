package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ToggleWheelExtenderCommand;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;

public class ComplexAutoPath extends ParallelCommandGroup {

    public ComplexAutoPath() {
        addCommands(
            new StraightThenRotateAutoPath(),
            new ToggleWheelExtenderCommand(WheelControllerSubsystem.getInstance(), CollisionAvoidanceSubsystem.getInstance())
        );
    }
}
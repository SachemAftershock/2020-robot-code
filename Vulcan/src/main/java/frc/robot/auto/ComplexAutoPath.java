package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ComplexAutoPath extends ParallelCommandGroup {

    public ComplexAutoPath() {
        addCommands(
            new StraightThenRotateAutoPath()
        );
    }
}
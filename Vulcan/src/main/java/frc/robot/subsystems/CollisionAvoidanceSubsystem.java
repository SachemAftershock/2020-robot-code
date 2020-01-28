package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Ultrasonic;

public class CollisionAvoidanceSubsystem extends SubsystemBase {

private Ultrasonic ultrasonic;

    public CollisionAvoidanceSubsystem() {
        ultrasonic = new Ultrasonic(0, 1);
        addChild("Ultrasonic",ultrasonic);
    }

    @Override
    public void periodic() {
    }
}


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionAlignmentSubsystem extends SubsystemBase {

    private static VisionAlignmentSubsystem mInstance;

    public VisionAlignmentSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public static VisionAlignmentSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new VisionAlignmentSubsystem();
        }
        return mInstance;
    }
}


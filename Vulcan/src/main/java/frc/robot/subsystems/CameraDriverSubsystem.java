package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraDriverSubsystem extends SubsystemBase {

    private static CameraDriverSubsystem mInstance;

    public CameraDriverSubsystem() {
    }

    public static CameraDriverSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CameraDriverSubsystem();
        }
        return mInstance;
    }
}


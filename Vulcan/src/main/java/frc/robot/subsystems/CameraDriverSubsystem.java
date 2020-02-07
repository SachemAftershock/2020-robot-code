package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraDriverSubsystem extends SubsystemBase implements SubsystemInterface {

    private static CameraDriverSubsystem mInstance;

    public CameraDriverSubsystem() {
    }

    @Override
    public void init() {
    }

    public synchronized static CameraDriverSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CameraDriverSubsystem();
        }
        return mInstance;
    }
}


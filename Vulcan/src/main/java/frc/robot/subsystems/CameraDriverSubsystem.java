package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraDriverSubsystem extends SubsystemBase implements SubsystemInterface {

    private boolean mUSBCameraEnabled;

    private static CameraDriverSubsystem mInstance;
    //Cameras are currently run by Python script in CameraStreamer Folder
    //If needed to plug camera directly into RoboRIO, use this Subsystem
    private CameraDriverSubsystem() {
        mUSBCameraEnabled = false;
    }

    @Override
    public void init() {
        if(mUSBCameraEnabled) {
            CameraServer.getInstance().startAutomaticCapture();
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("USB Camera Enabled", mUSBCameraEnabled);
    }

    public synchronized static CameraDriverSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CameraDriverSubsystem();
        }
        return mInstance;
    }
}


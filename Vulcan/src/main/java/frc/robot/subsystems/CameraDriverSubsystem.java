package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AftershockSubsystem;

/**
 * Class to enable USB Camera directly through the RoboRIO
 * 
 * @author Shreyas Prasad
 */
public class CameraDriverSubsystem extends AftershockSubsystem {

    private boolean mUSBCameraEnabled;

    private static CameraDriverSubsystem mInstance;
    //Cameras are currently run by Python script in CameraStreamer Folder
    //If needed to plug camera directly into RoboRIO, use this Subsystem
    private CameraDriverSubsystem() {
        super();
        setName("Camera Driver Subsystem");
        mUSBCameraEnabled = false;
    }

    @Override
    public void initialize() {
        if(mUSBCameraEnabled) {
            CameraServer.getInstance().startAutomaticCapture();
        }
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putData(getInstance());
        //SmartDashboard.putBoolean("USB Cam Enabled", mUSBCameraEnabled);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized static CameraDriverSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CameraDriverSubsystem();
        }
        return mInstance;
    }
}


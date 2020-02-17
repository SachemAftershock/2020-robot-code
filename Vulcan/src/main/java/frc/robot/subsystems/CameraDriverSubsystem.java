package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class to enable USB Camera directly through the RoboRIO
 * 
 * @author Shreyas Prasad
 */
public class CameraDriverSubsystem extends SubsystemBase implements SubsystemInterface {

    private BooleanSupplier mUSBCameraEnabled;

    private static CameraDriverSubsystem mInstance;
    //Cameras are currently run by Python script in CameraStreamer Folder
    //If needed to plug camera directly into RoboRIO, use this Subsystem
    private CameraDriverSubsystem() {
        setName("Camera Driver Subsystem");
        mUSBCameraEnabled = () -> false;
    }

    @Override
    public void init() {
        if(mUSBCameraEnabled.getAsBoolean()) {
            CameraServer.getInstance().startAutomaticCapture();
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putBoolean("USB Cam Enabled", mUSBCameraEnabled.getAsBoolean());
    }

    @Override
    public void runTest() {
        SmartDashboard.putData("Toggle USB Cam Enabled", 
            new ConditionalCommand((new InstantCommand(() -> mUSBCameraEnabled = () -> false)),(new InstantCommand(() -> mUSBCameraEnabled = () -> true)), mUSBCameraEnabled));
    }

    public synchronized static CameraDriverSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CameraDriverSubsystem();
        }
        return mInstance;
    }
}


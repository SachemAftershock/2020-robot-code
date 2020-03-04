package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class to determine the absolute positon of the Robot on the field using SLAM from the rotational YDLidar
 * @author Shreyas Prasad
 */
public class AbsoluteFieldPositionDeviceSubsystem extends SubsystemBase implements SubsystemInterface {

    private static AbsoluteFieldPositionDeviceSubsystem mInstance;
    
    private AbsoluteFieldPositionDeviceSubsystem() {
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putData(getInstance());
    }
    
    @Override
    public void runTest() {  
    }
    
    /**
     * @return AbsoluteFieldPositionDeviceSubsystem Singleton Instance
     */
    public synchronized static AbsoluteFieldPositionDeviceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new AbsoluteFieldPositionDeviceSubsystem();
        }
        return mInstance;
    }
}


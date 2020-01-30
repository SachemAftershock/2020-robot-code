package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsoluteFieldPositionDeviceSubsystem extends SubsystemBase {

    private static AbsoluteFieldPositionDeviceSubsystem mInstance;
    //This is intended for the rotational LIDAR YDLidar
    
    public AbsoluteFieldPositionDeviceSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public static AbsoluteFieldPositionDeviceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new AbsoluteFieldPositionDeviceSubsystem();
        }
        return mInstance;
    }
}


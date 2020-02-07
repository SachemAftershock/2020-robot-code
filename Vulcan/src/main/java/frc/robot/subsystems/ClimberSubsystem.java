package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ClimberSubsystem mInstance;
    
    public ClimberSubsystem() {
    }

    @Override
    public void init() {
    }

    public synchronized static ClimberSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ClimberSubsystem();
        }
        return mInstance;
    }
}


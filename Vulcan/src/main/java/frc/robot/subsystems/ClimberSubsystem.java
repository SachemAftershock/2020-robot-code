package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private static ClimberSubsystem mInstance;
    
    public ClimberSubsystem() {
    }

    public static ClimberSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ClimberSubsystem();
        }
        return mInstance;
    }
}


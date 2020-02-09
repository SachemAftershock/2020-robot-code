package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerSubsystem extends SubsystemBase implements SubsystemInterface {

    private static PowerSubsystem mInstance;

    private PowerDistributionPanel mPdp;
    //TODO: Add print for system information, start printing system info by command
    private PowerSubsystem() {
        mPdp = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel",mPdp);
    }

    @Override
    public void init() {
    }

    public synchronized static PowerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new PowerSubsystem();
        }
        return mInstance;
    }
}


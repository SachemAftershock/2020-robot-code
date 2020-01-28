package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerSubsystem extends SubsystemBase {

    private PowerDistributionPanel mPdp;
    //TODO: Find out if we can use this for anything useful
    public PowerSubsystem() {
        mPdp = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel",mPdp);
    }
}


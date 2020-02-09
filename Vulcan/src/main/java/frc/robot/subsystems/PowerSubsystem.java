package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerSubsystem extends SubsystemBase implements SubsystemInterface {

    private static PowerSubsystem mInstance;

    private PowerDistributionPanel mPdp;
    
    private PowerSubsystem() {
        mPdp = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel",mPdp);
    }

    @Override
    public void init() {
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Input Voltage", mPdp.getVoltage());
        SmartDashboard.putNumber("Temperature", mPdp.getTemperature());
        SmartDashboard.putNumber("Total Current", mPdp.getTotalCurrent());
        SmartDashboard.putNumber("Total Energy", mPdp.getTotalEnergy());
        SmartDashboard.putNumber("Total Power", mPdp.getTotalPower());
        for(int i=0;i<16;i++) {
            SmartDashboard.putNumber("Channel " +  i + " Current", mPdp.getCurrent(i));
        }
    }

    public synchronized static PowerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new PowerSubsystem();
        }
        return mInstance;
    }
}


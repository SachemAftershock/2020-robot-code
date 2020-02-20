package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to provide data on the flow of electricity around the Robot
 * 
 * @author Shreyas Prasad
 */
public class PowerSubsystem extends SubsystemBase implements SubsystemInterface {

    private static PowerSubsystem mInstance;

    private PowerDistributionPanel mPdp;
    
    /**
     * Constructor for PowerSubsystem Class
     */
    private PowerSubsystem() {
        mPdp = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel",mPdp);
    }

    @Override
    public void init() {
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Input Voltage", mPdp.getVoltage());
        SmartDashboard.putNumber("Temperature", mPdp.getTemperature());
        SmartDashboard.putNumber("Total Current", mPdp.getTotalCurrent());
        SmartDashboard.putNumber("Total Energy", mPdp.getTotalEnergy());
        SmartDashboard.putNumber("Total Power", mPdp.getTotalPower());
        for(int i=0;i<16;i++) {
            SmartDashboard.putNumber("Channel " +  i + " Current", mPdp.getCurrent(i));
        }
    }

    @Override
    public void runTest() {
        // TODO Auto-generated method stub
    }

    /**
     * @return PowerSubsystem Singleton Instance
     */
    public synchronized static PowerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new PowerSubsystem();
        }
        return mInstance;
    }
}


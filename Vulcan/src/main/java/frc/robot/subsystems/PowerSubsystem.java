package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
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

    private Compressor mCompressor;
    
    /**
     * Constructor for PowerSubsystem Class
     */
    private PowerSubsystem() {
        mPdp = new PowerDistributionPanel(0);
        addChild("PowerDistributionPanel",mPdp);

        mCompressor = new Compressor();
    }

    @Override
    public void init() {
        mCompressor.setClosedLoopControl(true);
        mCompressor.start();
    }

    public void startCompressor() {
        mCompressor.start();
    }

    public void stopCompressor() {
        mCompressor.stop();
    }

    @Override
    public void outputTelemetry() {
        //SmartDashboard.putData(getInstance());
        //SmartDashboard.putNumber("Input Voltage", mPdp.getVoltage());
        //SmartDashboard.putNumber("Temperature", mPdp.getTemperature());
        //SmartDashboard.putNumber("Total Current", mPdp.getTotalCurrent());
        //SmartDashboard.putNumber("Total Energy", mPdp.getTotalEnergy());
        //SmartDashboard.putNumber("Total Power", mPdp.getTotalPower());
        /*
        for(int i=0;i<16;i++) {
            SmartDashboard.putNumber("Channel " +  i + " Current", mPdp.getCurrent(i));
        }
        */
        SmartDashboard.putNumber("Channel 3 Current", mPdp.getCurrent(3));
    }

    @Override
    public void runTest() {
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


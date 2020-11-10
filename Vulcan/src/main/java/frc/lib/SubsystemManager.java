package frc.lib;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class to Manage All Subsystems
 * 
 * @author Shreyas Prasad
 */
public class SubsystemManager {
    
    private static SubsystemManager mInstance;

    private List<AftershockSubsystem> mAllSubsystems;

    /**
     * Subsystem Manager Constructor
     */
    private SubsystemManager() {
    }

    /**
     * Sets List of Subsystems to Manage
     * 
     * @param allSubsystems List of all Subsystems to Manage
     */
    public void setSubsystems(AftershockSubsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    /**
     * Initializes all Subsystems
     */
    public void initialize() {
        mAllSubsystems.forEach(AftershockSubsystem::initialize);
    }

    /**
     * Outputs Telemetry of all Subsystems
     */
    public void outputTelemetry() {
        mAllSubsystems.forEach(AftershockSubsystem::outputTelemetry);
    }

    /**
     * Checks all Subsystems and Reports all Subsystems that fail systems check
     * 
     * @return true if all subsystems pass systems check; false otherwise
     */
    public boolean checkSystems() {
        boolean ret = true;
        for(AftershockSubsystem subsystem : mAllSubsystems) {
            boolean subsystemPassedCheck = subsystem.checkSystem();
            if(!subsystemPassedCheck) {
                DriverStation.reportWarning(subsystem.getName() + " FAILED SYSTEM CHECK", false);
            }
            ret &= subsystemPassedCheck;
        }
        return ret;
    }

    /**
     * Gets list of all Subsystems
     * 
     * @return List of all Subsystems
     */
    public List<AftershockSubsystem> getSubsystems() {
        return mAllSubsystems;
    }

    /**
     * Gets Singleton Instance of Subsystem Manager
     * 
     * @return Singleton Instance of Subsystem Manager
     */
    public static SubsystemManager getInstance() {
        if(mInstance == null) {
            mInstance = new SubsystemManager();
        }
        return mInstance;
    }
}
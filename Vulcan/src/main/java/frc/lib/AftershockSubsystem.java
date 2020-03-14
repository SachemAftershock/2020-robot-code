package frc.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base Subsystem Class
 * 
 * @author Shreyas Prasad
 */
public abstract class AftershockSubsystem extends SubsystemBase {

    public AftershockSubsystem() {
        super();
    }

    /**
     * Checks Subsystem
     * 
     * @return <i> true </i> if subsystem has no errors; <i> false </i> otherwise
     */
    public boolean checkSystem() {
        return true;
    }

    /**
     * Subsystem Intialization
     */
    public abstract void initialize();

    /**
     * Output Data for each respective Subsystem to SmartDashboard
     */
    public abstract void outputTelemetry();
}
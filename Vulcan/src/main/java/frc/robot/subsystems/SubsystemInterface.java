package frc.robot.subsystems;

/**
 * Interface for consistent structure among all Subsystems
 * 
 * @author Shreyas Prasad
 */
public interface SubsystemInterface {
    /**
     * Subsystem Intialization, runs when
     * <ul>
     * <li> Robot Powered On
     * <li> Autonomous Begins
     * </ul>
     */
    public void init();
    
    /**
     * Output Data for each respective Subsystem
     */
    public void outputTelemetry();

    /**
     * Checks Subsystem
     * 
     * @return <i> true </i> if subsystem has no errors; <i> false </i> otherwise
     */
    public boolean checkSystem();
}
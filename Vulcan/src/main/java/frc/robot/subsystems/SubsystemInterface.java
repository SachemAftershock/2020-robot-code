package frc.robot.subsystems;

/**
 * Interface for consistent structure among all Subsystems
 */
public interface SubsystemInterface {
    public void init();
    
    /**
     * Output Telemetry Data for each respective Subsystem
     */
    public void outputTelemetry();
}
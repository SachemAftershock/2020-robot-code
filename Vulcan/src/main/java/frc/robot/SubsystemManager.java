package frc.robot;

import java.util.List;

import frc.lib.AftershockSubsystem;

public class SubsystemManager {
    
    private final List<AftershockSubsystem> mAllSubsystems;

    public SubsystemManager(List<AftershockSubsystem> subsystems) {
        mAllSubsystems = subsystems;
    }

    public void initialize() {
        mAllSubsystems.forEach(AftershockSubsystem::initialize);
    }

    public void outputTelemetry() {
        mAllSubsystems.forEach(AftershockSubsystem::outputTelemetry);
    }

    public boolean checkSystems() {
        boolean ret = true;
        for(AftershockSubsystem subsystem : mAllSubsystems) {
            ret &= subsystem.checkSystem();
        }
        return ret;
    }
}
package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class AbsoluteFieldPositionDeviceSubsystem extends SubsystemBase {
    //This is intended for the rotational LIDAR YDLidar
    public AbsoluteFieldPositionDeviceSubsystem() {
        //TODO: 
        //CommandScheduler.getInstance().setDefaultCommand(subsystem, defaultCommand);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

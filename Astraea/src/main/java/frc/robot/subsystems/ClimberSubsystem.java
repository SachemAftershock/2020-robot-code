package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

public class ClimberSubsystem extends SubsystemBase {
    
    private VictorSP climberShaftMotorController;
    private PWMVictorSPX climberStafferMotorController;

    public ClimberSubsystem() {

        climberShaftMotorController = new VictorSP(5);
        addChild("Climber Shaft Motor Controller",climberShaftMotorController);
        climberShaftMotorController.setInverted(false);
            
        climberStafferMotorController = new PWMVictorSPX(6);
        addChild("Climber Staffer Motor Controller",climberStafferMotorController);
        climberStafferMotorController.setInverted(false);
            
        //TODO: 
        //CommandScheduler.getInstance().setDefaultCommand(subsystem, defaultCommand);
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setClimberShaftSpeed(double speed) {
        climberShaftMotorController.set(speed);
    }

    public void setClimberStrafeSpeed(double speed) {
        climberStafferMotorController.set(speed);
    }

}


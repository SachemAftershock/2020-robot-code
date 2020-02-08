package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SubsystemInterface;

public class Robot extends TimedRobot {

    private Command mAutonomousCommand;
    private RobotContainer mRobotContainer;
    private Compressor mCompressor;

    private AutoSelector mAutoSelector;


    @Override
    public void robotInit() {
        mRobotContainer = RobotContainer.getInstance();

        for(SubsystemInterface subsystem : mRobotContainer.getSubsystemList()) {
            subsystem.init();
        }
        
        mCompressor = new Compressor();
        mCompressor.setClosedLoopControl(true);
        mCompressor.start();

        mAutoSelector.selectAuto();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        mRobotContainer.periodic();
    }

    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        mAutoSelector.selectAuto();
    }

    @Override
    public void autonomousInit() {
        mAutonomousCommand = mAutoSelector.getSelectedAutoCommand();
        if (mAutonomousCommand != null) {
          mAutonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (mAutonomousCommand != null) mAutonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
      CommandScheduler.getInstance().cancelAll();
    }
  
    @Override
    public void testPeriodic() {
    }
}

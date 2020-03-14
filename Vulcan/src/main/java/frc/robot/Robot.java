package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Limelight.LightMode;
import frc.robot.subsystems.LimelightManagerSubsystem;

public class Robot extends TimedRobot {

    private Command mAutonomousCommand;

    private RobotContainer mRobotContainer;
    private SubsystemManager mSubsystemManager;

    private AutoSelector mAutoSelector;

    @Override
    public void robotInit() {
        mRobotContainer = RobotContainer.getInstance();
        mSubsystemManager = mRobotContainer.getSubsystemManager();
        mRobotContainer.initialize();
        mSubsystemManager.initialize();

        mAutoSelector = new AutoSelector();
        mAutoSelector.selectAuto();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        mRobotContainer.periodic();
    }

    @Override
    public void disabledInit(){
        LimelightManagerSubsystem.getInstance().setAllLightMode(LightMode.eOff);
    }

    @Override
    public void disabledPeriodic() {
        mAutoSelector.selectAuto();
    }

    @Override
    public void autonomousInit() {
        mSubsystemManager.initialize();

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
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        mSubsystemManager.initialize(); //TODO: Remove before comp
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }
  
    @Override
    public void testPeriodic() { 
    }
}

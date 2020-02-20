package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Limelight.LightMode;
import frc.robot.subsystems.LimelightManagerSubsystem;
import frc.robot.subsystems.SubsystemInterface;

public class Robot extends TimedRobot {

    private Command mAutonomousCommand;
    private RobotContainer mRobotContainer;
    private Compressor mCompressor;

    private AutoSelector mAutoSelector;


    @Override
    public void robotInit() {
        mRobotContainer = RobotContainer.getInstance();
        mRobotContainer.init();

        RobotContainer.getInstance().getSubsystemList().forEach(SubsystemInterface::init);
        
        mCompressor = new Compressor();
        mCompressor.setClosedLoopControl(true);
        mCompressor.start();

        mAutoSelector = new AutoSelector();
        mAutoSelector.selectAuto();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        mRobotContainer.periodic();

        try {
            mRobotContainer.getSubsystemList().forEach(SubsystemInterface::outputTelemetry);
        } catch(Throwable t) {
            System.out.println(t);
        }
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
        RobotContainer.getInstance().getSubsystemList().forEach(SubsystemInterface::init);

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
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }
  
    @Override
    public void testPeriodic() { 
        try {
            mRobotContainer.getSubsystemList().forEach(SubsystemInterface::runTest);
        } catch(Throwable t) {
            System.out.println(t);
        }
    }
}

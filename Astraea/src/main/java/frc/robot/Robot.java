package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    SendableChooser<Command> chooser = new SendableChooser<>();

    /*
    public static DrivebaseSubsystem drivebase;
    public static CameraDriverForwardSubsystem cameraDriverForward;
    public static CameraDriverAftSubsystem cameraDriverAft;
    public static VisionAlignmentSubsystem visionAlignment;
    public static GyroscopeSubsystem gyroscope;
    public static CollisionAvoidanceSubsystem collisionAvoidance;
    public static ShooterSubsystem shooter;
    public static BallFloorHarvestorSubsystem ballFloorHarvestor;
    public static BallContainerSubsystem ballContainer;
    public static BallLoadingDockHighHarvestorSubsystem ballLoadingDockHighHarvestor;
    public static AbsoluteFieldPositionDeviceSubsystem absoluteFieldPositionDevice;
    public static ColorWheelControllerSubsystem colorWheelController;
    public static PowerSubsystem power;
    public static CompressedAirSubsystem compressedAir;
    public static ClimberSubsystem climber;
*/

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

        // Add commands to Autonomous Sendable Chooser
        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Auto mode", chooser);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
//TODO: reconcile chooser vs. container- keep old style chooser, but do new way.
//        m_autonomousCommand = chooser.getSelected();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
          m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        //CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        //handled in robotPeriodic now.
        //CommandScheduler.getInstance().run();
    }

    @Override
    public void testInit() {
      // Cancels all running commands at the start of test mode.
      CommandScheduler.getInstance().cancelAll();
    }
  
    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
  
}

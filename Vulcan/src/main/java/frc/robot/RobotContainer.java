package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AftershockSubsystem;
import frc.lib.AftershockXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.drive.ToggleCollisionAvoidanceCommand;
import frc.robot.commands.drive.ToggleDrivebaseGearingCommand;
import frc.robot.commands.drive.ToggleManualDriveInversionCommand;
import frc.robot.commands.drive.TogglePrecisionDrivingCommand;
import frc.robot.commands.superstructure.ManualEjectCommand;
import frc.robot.commands.superstructure.ManualIngestCommand;
import frc.robot.commands.superstructure.SetFeedModeCommand;
import frc.robot.commands.superstructure.SetIdleModeCommand;
import frc.robot.commands.superstructure.StopManualDriveCommand;
import frc.robot.commands.wheelcontroller.ToggleWheelExtenderCommand;
import frc.robot.commands.wheelcontroller.WheelColorControlCommand;
import frc.robot.commands.wheelcontroller.WheelRotateControlCommand;
import frc.robot.subsystems.CameraDriverSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightManagerSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;
import frc.robot.subsystems.LEDSubsystem.SystemState;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

/**
 * Class to instantiate the structure of the Robot
 * <p> 
 * Instantiates Subsystems & Operator Control Scheme
 * 
 * @author Shreyas Prasad
 */
public class RobotContainer {

    private static RobotContainer mInstance;

    private final AftershockXboxController mControllerPrimary = new AftershockXboxController(ControllerConstants.kControllerPrimaryId);
    private final AftershockXboxController mControllerSecondary = new AftershockXboxController(ControllerConstants.kControllerSecondaryId);

    private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
    private final CameraDriverSubsystem mDriverCamera = CameraDriverSubsystem.getInstance();
    private final ClimberSubsystem mClimber = ClimberSubsystem.getInstance();
    private final CollisionAvoidanceSubsystem mCollisionAvoidance = CollisionAvoidanceSubsystem.getInstance();
    private final WheelControllerSubsystem mColorWheelController = WheelControllerSubsystem.getInstance();
    private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
    private final PowerSubsystem mPower = PowerSubsystem.getInstance();
    private final ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
    private final StorageSubsystem mStorage = StorageSubsystem.getInstance();
    //private final TurretSubsystem mTurret = TurretSubsystem.getInstance(); //TODO: Change when Turret Implemented
    private final SuperstructureSubsystem mSuperstructure = SuperstructureSubsystem.getInstance();
    private final LEDSubsystem mLED = LEDSubsystem.getInstance();
    private final LimelightManagerSubsystem mLimelightManager = LimelightManagerSubsystem.getInstance();
    private final ArrayList<AftershockSubsystem> mAllSubsystems;
    private final SubsystemManager mSubsystemManager;

    // Primary Controller
    private JoystickButton bToggleDriveGear;
    private JoystickButton bToggleDriveInversion;
    private JoystickButton bToggleCollisionAvoidance;
    private JoystickButton bTogglePrecisionDrive;
    private JoystickButton bDriveClimbLifter;
    private JoystickButton bRaiseClimbElevator;
    private JoystickButton bLowerClimbElevator;
    private JoystickButton bClearCommandQueuePrimary;

    // Secondary Controller
    //private JoystickButton bToggleTurretAutoTargeting; //TODO: Change when Turret Implemented
    private JoystickButton bStartWheelPositionControl;
    private JoystickButton bStartWheelColorTargeting;
    private JoystickButton bDeployWheelController;
    private JoystickButton bToggleSuperstructureMode;
    private JoystickButton bClearCommandQueueSecondary;
    private JoystickButton bEjectIntakeAndStorage;
    private JoystickButton bIngestIntakeAndStorage;

    /**
     * Constructor for RobotCotainer Class
     */
    private RobotContainer() {
        mAllSubsystems = new ArrayList<AftershockSubsystem>();
        mAllSubsystems.add(mDriverCamera);
        mAllSubsystems.add(mClimber);
        mAllSubsystems.add(mCollisionAvoidance);
        mAllSubsystems.add(mDrive);
        mAllSubsystems.add(mIntake);
        mAllSubsystems.add(mLED);
        mAllSubsystems.add(mLimelightManager);
        mAllSubsystems.add(mPower);
        mAllSubsystems.add(mShooter);
        mAllSubsystems.add(mStorage);
        mAllSubsystems.add(mSuperstructure);
        //mSubsystems.add(mTurret); //TODO: Change when Turret Implemented
        mAllSubsystems.add(mColorWheelController);
        mSubsystemManager = new SubsystemManager(mAllSubsystems);

        configureButtonBindings();
        CommandScheduler.getInstance().setDefaultCommand(mDrive, new ManualDriveCommand(mDrive, mControllerPrimary));
    }

    /**
     * Maps Buttons on Primary & Secondary Controllers to Commands
     */
    private void configureButtonBindings() {
        // PRIMARY CONTROLLER
        bToggleDriveGear = new JoystickButton(mControllerPrimary, XboxController.Button.kA.value);
        bToggleDriveGear.whenPressed(new ToggleDrivebaseGearingCommand(mDrive));

        bTogglePrecisionDrive = new JoystickButton(mControllerPrimary, XboxController.Button.kX.value);
        bTogglePrecisionDrive.whenPressed(new TogglePrecisionDrivingCommand(mDrive));

        bToggleDriveInversion = new JoystickButton(mControllerPrimary, XboxController.Button.kB.value);
        bToggleDriveInversion.whenPressed(new ToggleManualDriveInversionCommand(mDrive));


        bToggleCollisionAvoidance = new JoystickButton(mControllerPrimary, XboxController.Button.kBack.value);
        bToggleCollisionAvoidance.whenPressed(new ToggleCollisionAvoidanceCommand(mCollisionAvoidance, mControllerPrimary));

        bClearCommandQueuePrimary = new JoystickButton(mControllerPrimary, XboxController.Button.kStart.value);
        bClearCommandQueuePrimary.whenPressed(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        bDriveClimbLifter = new JoystickButton(mControllerPrimary, XboxController.Button.kY.value);
        bDriveClimbLifter.whenPressed(new InstantCommand(() -> mClimber.driveLifter()))
                .whenReleased(new InstantCommand(() -> mClimber.stopLifter()));

        bRaiseClimbElevator = new JoystickButton(mControllerPrimary, XboxController.Button.kBumperRight.value);
        bRaiseClimbElevator.whenReleased(new InstantCommand(() -> mClimber.stopElevator()))
                            .whenPressed(new InstantCommand(() -> mClimber.forwardElevator())/*new RaiseElevatorLevelCommand(mClimber)*/);

        bLowerClimbElevator = new JoystickButton(mControllerPrimary, XboxController.Button.kBumperLeft.value);
        bLowerClimbElevator.whenReleased(new InstantCommand(() -> mClimber.stopElevator()))
                            .whenPressed(new InstantCommand(() -> mClimber.reverseElevator())/*new LowerElevatorLevelCommand(mClimber)*/);

        // SECONDARY CONTROLLER
        bToggleSuperstructureMode = new JoystickButton(mControllerSecondary, XboxController.Button.kA.value);
        bToggleSuperstructureMode.whenReleased(new SetIdleModeCommand(mSuperstructure)).whenPressed(new SetFeedModeCommand(mSuperstructure));

        bStartWheelPositionControl = new JoystickButton(mControllerSecondary, XboxController.Button.kX.value);
        bStartWheelPositionControl.whenPressed(new WheelRotateControlCommand(mColorWheelController));

        bStartWheelColorTargeting = new JoystickButton(mControllerSecondary, XboxController.Button.kY.value);
        bStartWheelColorTargeting.whenPressed(new WheelColorControlCommand(mColorWheelController));

        bDeployWheelController = new JoystickButton(mControllerSecondary, XboxController.Button.kB.value);
        bDeployWheelController.whenPressed(new ToggleWheelExtenderCommand(mColorWheelController, mCollisionAvoidance));

        //bToggleTurretAutoTargeting = new JoystickButton(mControllerSecondary, XboxController.Button.kBack.value); //TODO: Change when Turret Implemented
        //bToggleTurretAutoTargeting.whenPressed(new InstantCommand(() -> mTurret.toggleAutoTargetingEnabled()));

        bClearCommandQueueSecondary = new JoystickButton(mControllerSecondary, XboxController.Button.kStart.value);
        bClearCommandQueueSecondary.whenPressed(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

        bEjectIntakeAndStorage = new JoystickButton(mControllerSecondary, XboxController.Button.kBumperLeft.value);
        bEjectIntakeAndStorage.whenReleased(new StopManualDriveCommand(mIntake, mStorage))
                                .whenPressed(new ManualEjectCommand(mIntake, mStorage));

        bIngestIntakeAndStorage = new JoystickButton(mControllerSecondary, XboxController.Button.kBumperRight.value);
        bIngestIntakeAndStorage.whenReleased(new StopManualDriveCommand(mIntake, mStorage))
                                .whenPressed(new ManualIngestCommand(mIntake, mStorage));
    }

    /**
     * RobotContainer Initialization, Runs at Robot Powered On
     */
    public void initialize() {
        mLED.forceSystemState(SystemState.eInit);
    }


    /**
     * Runs Periodically
     */
    public void periodic() {
        if(mControllerPrimary.getDPadActive() && !mDrive.isAutoRotateRunning()) {
            CommandScheduler.getInstance().schedule(new RotateDriveCommand(mDrive, mControllerPrimary.getPOV()));
        }

        if(mControllerSecondary.getTriggerPressed(Hand.kLeft)) {
            mSuperstructure.setMode(SuperstructureMode.eArmed);
        } else {
            if(mSuperstructure.getCurrentMode() == SuperstructureMode.eArmed && !mControllerSecondary.getTriggerHeld(Hand.kLeft)) {
                mSuperstructure.setMode(SuperstructureMode.eIdle);
            }
        }

        if(mControllerSecondary.getTriggerHeld(Hand.kRight)) {
            mSuperstructure.authorizeShot();
        } else {
            mSuperstructure.deauthorizeShot();
        }

        if(!mColorWheelController.isCommandRunning()) {
            mColorWheelController.manualControl(mControllerSecondary.getDeadbandX(Hand.kRight));
        }

        //TODO: Change when Turret Implemented
        //mTurret.manualControl(mControllerSecondary.getDeadbandX(Hand.kLeft));
    }
    
    /**
     * Gets Xbox Controller for the Primary Driver, tasked with driving the robot
     * 
     * @return Primary Xbox Controller
     */
    public AftershockXboxController getControllerPrimary() {
        return mControllerPrimary;
    }

    /**
     * Gets Xbox Controller for the Secondary Driver, tasked with controlling all mechanisms
     * 
     * @return Secondary Xbox Controller
     */
    public AftershockXboxController getControllerSecondary() {
        return mControllerSecondary;
    }

    public SubsystemManager getSubsystemManager() {
        return mSubsystemManager;
    }

    /**
     * @return RobotContainer Singleton Instance
     */
    public synchronized static RobotContainer getInstance() {
        if(mInstance == null) {
            mInstance = new RobotContainer();
        }
        return mInstance;
    }
}


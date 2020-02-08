package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.ToggleCollisionAvoidanceCommand;
import frc.robot.commands.drive.ToggleDrivebaseGearingCommand;
import frc.robot.commands.drive.TogglePrecisionDrivingCommand;
import frc.robot.commands.superstructure.SetArmedModeCommand;
import frc.robot.commands.superstructure.SetIdleModeCommand;
import frc.robot.commands.superstructure.shooter.AuthorizeShotCommand;
import frc.robot.commands.superstructure.shooter.DeauthorizeShotCommand;
import frc.robot.commands.wheelcontroller.WheelColorControlCommand;
import frc.robot.commands.wheelcontroller.WheelPositionControlCommand;
import frc.robot.subsystems.AbsoluteFieldPositionDeviceSubsystem;
import frc.robot.subsystems.CameraDriverSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SubsystemInterface;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;

public class RobotContainer {

    private static RobotContainer mInstance;

    private final XboxController mControllerPrimary = new XboxController(Constants.kControllerPrimaryId);
    private final XboxController mControllerSecondary = new XboxController(Constants.kControllerSecondaryId);

    private final AbsoluteFieldPositionDeviceSubsystem mAbsoluteFieldPosition = AbsoluteFieldPositionDeviceSubsystem.getInstance();
    private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
    private final CameraDriverSubsystem mDriverCamera = CameraDriverSubsystem.getInstance();
    private final ClimberSubsystem mClimber = ClimberSubsystem.getInstance();
    private final CollisionAvoidanceSubsystem mCollisionAvoidanceSubsystem = CollisionAvoidanceSubsystem.getInstance();
    private final WheelControllerSubsystem mColorWheelController = WheelControllerSubsystem.getInstance();
    private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
    private final PowerSubsystem mPower = PowerSubsystem.getInstance();
    private final ShooterSubsystem mShooter =  ShooterSubsystem.getInstance();
    private final TurretSubsystem mTurret = TurretSubsystem.getInstance();
    private final SuperstructureSubsystem mSuperstructure = SuperstructureSubsystem.getInstance();
    private final ArrayList<SubsystemInterface> mSubsystems;
    
    private JoystickButton bToggleDriveGear;
    private JoystickButton bToggleCollisionAvoidance;
    private JoystickButton bStartWheelPositionControl;
    private JoystickButton bStartWheelColorTargeting;
    private JoystickButton bTogglePrecisionDrive;
    private JoystickButton bStartShooter;
    private JoystickButton bShooterAuthorized;
    private JoystickButton bToggleTurretAutoTargeting;

    public RobotContainer() {
        mSubsystems = new ArrayList<SubsystemInterface>();
        mSubsystems.add(mAbsoluteFieldPosition);
        mSubsystems.add(mIntake);
        mSubsystems.add(mDriverCamera);
        mSubsystems.add(mClimber);
        mSubsystems.add(mCollisionAvoidanceSubsystem);
        mSubsystems.add(mDrive);
        mSubsystems.add(mPower);
        mSubsystems.add(mShooter);
        mSubsystems.add(mTurret);
        mSubsystems.add(mSuperstructure);
        
        configureButtonBindings();
        CommandScheduler.getInstance().setDefaultCommand(mDrive, new ManualDriveCommand(mDrive, mControllerPrimary));
    }

    private void configureButtonBindings() {
        //PRIMARY CONTROLLER
        bToggleDriveGear = new JoystickButton(mControllerPrimary, XboxController.Button.kA.value);
        bToggleDriveGear.whenPressed(new ToggleDrivebaseGearingCommand(mDrive, mControllerPrimary));

        bTogglePrecisionDrive = new JoystickButton(mControllerPrimary, XboxController.Button.kB.value);
        bTogglePrecisionDrive.whenPressed(new TogglePrecisionDrivingCommand(mDrive));

        bToggleCollisionAvoidance = new JoystickButton(mControllerPrimary, XboxController.Button.kBack.value);
        bToggleCollisionAvoidance.whenPressed(new ToggleCollisionAvoidanceCommand(mCollisionAvoidanceSubsystem, mControllerPrimary));
        
        //SECONDARY CONTROLLER
        bStartWheelPositionControl = new JoystickButton(mControllerSecondary, XboxController.Button.kX.value);
        bStartWheelPositionControl.whenPressed(new WheelPositionControlCommand(mColorWheelController));

        bStartWheelColorTargeting = new JoystickButton(mControllerSecondary, XboxController.Button.kB.value);
        bStartWheelColorTargeting.whenPressed(new WheelColorControlCommand(mColorWheelController));

        bStartShooter = new JoystickButton(mControllerSecondary, XboxController.Axis.kLeftTrigger.value);
        bStartShooter
        .whenReleased(new SetIdleModeCommand(mSuperstructure))
        .whileActiveOnce(new SetArmedModeCommand(mSuperstructure));

        bShooterAuthorized = new JoystickButton(mControllerSecondary, XboxController.Axis.kRightTrigger.value);
        bShooterAuthorized
        .whenReleased(new DeauthorizeShotCommand(mSuperstructure))
        .whileActiveOnce(new AuthorizeShotCommand(mSuperstructure));

        bToggleTurretAutoTargeting = new JoystickButton(mControllerSecondary, XboxController.Button.kStart.value);
        bToggleTurretAutoTargeting.whenPressed(new InstantCommand(() -> mTurret.toggleAutoTargetingEnabled()));
    }

    public void periodic() {
        final double secondaryLeftXAxis = mControllerSecondary.getX(Hand.kLeft);
        mTurret.manualControl(Util.deadband(secondaryLeftXAxis, Constants.kDeadbandTolerance));
    }

    public XboxController getControllerPrimary() {
        return mControllerPrimary;
    }

    public XboxController getControllerSecondary() {
        return mControllerSecondary;
    }

    /**
     * @return ArrayList containing All Subsystems
     */
    public ArrayList<SubsystemInterface> getSubsystemList() {
        return mSubsystems;
    }

    public static RobotContainer getInstance() {
        if(mInstance == null) {
            mInstance = new RobotContainer();
        }
        return mInstance;
    }
}


package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

    private final XboxController mXboxControllerPrimary = new XboxController(Constants.kControllerPrimaryId);
    private final XboxController mXboxControllerSecondary = new XboxController(Constants.kControllerSecondaryId);

    private final AbsoluteFieldPositionDeviceSubsystem mAbsoluteFieldPosition = new AbsoluteFieldPositionDeviceSubsystem();
    private final IntakeSubsystem mIntake = new IntakeSubsystem();
    private final CameraDriverSubsystem mDriverCamera = new CameraDriverSubsystem();
    private final ClimberSubsystem mClimber = new ClimberSubsystem();
    private final CollisionAvoidanceSubsystem mCollisionAvoidanceSubsystem = new CollisionAvoidanceSubsystem();
    private final WheelControllerSubsystem mColorWheelController = new WheelControllerSubsystem();
    private final DriveSubsystem mDrive = new DriveSubsystem();
    private final PowerSubsystem mPower = new PowerSubsystem();
    private final ShooterSubsystem mShooter =  new ShooterSubsystem();
    private final TurretSubsystem mTurret = new TurretSubsystem();
    private final VisionAlignmentSubsystem mVisionAlignment = new VisionAlignmentSubsystem();

    private JoystickButton bToggleDriveGear;
    private JoystickButton bToggleShooterElevation;
    private JoystickButton bStartWheelPositionControl;
    private JoystickButton bStartWheelColorTargeting;
    private JoystickButton bTogglePrecisionDrive;
    private JoystickButton bToggleIntakeExtender;
    private JoystickButton bIngestIntake;
    private JoystickButton bEjectIntake;
    private JoystickButton bStartShooter;
    private JoystickButton jTurret;

    public RobotContainer() {
        configureButtonBindings();
        CommandScheduler.getInstance().setDefaultCommand(mDrive, new ManualDriveCommand(mDrive, mXboxControllerPrimary));
    }

    private void configureButtonBindings() {
        //PRIMARY CONTROLLER
        bToggleDriveGear = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kA.value);
        bToggleDriveGear.whenPressed(new ToggleDrivebaseGearingCommand(mDrive, mXboxControllerPrimary));

        bTogglePrecisionDrive = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kB.value);
        bTogglePrecisionDrive.whenPressed(new TogglePrecisionDrivingCommand(mDrive));
        //Not sure if I want primary driver to have this, but maybe it'll be better?
        bIngestIntake = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kBumperRight.value);
        bIngestIntake.whileHeld(new RunIntakeIngestCommand(mIntake));

        bEjectIntake = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kBumperLeft.value);
        bEjectIntake.whileHeld(new RunIntakeEjectCommand(mIntake));

        bToggleIntakeExtender = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kStart.value);
        bToggleIntakeExtender.whenPressed(new ToggleIntakeExtenderCommand(mIntake));
        
        //SECONDARY CONTROLLER
        bToggleShooterElevation = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kY.value);
        bToggleShooterElevation.whenPressed(new ToggleShooterElevationCommand(mShooter, mXboxControllerSecondary));

        bStartWheelPositionControl = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kX.value);
        bStartWheelPositionControl.whenPressed(new WheelPositionControlCommand(mColorWheelController));

        bStartWheelColorTargeting = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kB.value);
        bStartWheelColorTargeting.whenPressed(new WheelColorControlCommand(mColorWheelController));

        bStartShooter = new JoystickButton(mXboxControllerSecondary, XboxController.Axis.kLeftTrigger.value);
        bStartShooter.whileHeld(new ReachTargetRPM(mShooter));

        //TODO: Not sure if this will work as intended
        jTurret = new JoystickButton(mXboxControllerSecondary, XboxController.Axis.kLeftX.value);
        jTurret.whileActiveContinuous(new DriveTurretCommand(mTurret, mXboxControllerSecondary));

        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Toggle Drivebase Gearing", new ToggleDrivebaseGearingCommand(mDrive, mXboxControllerPrimary));
    }

    public XboxController getXboxControllerPrimary() {
        return mXboxControllerPrimary;
    }

    public XboxController getXboxControllerSecondary() {
        return mXboxControllerSecondary;
    }
}


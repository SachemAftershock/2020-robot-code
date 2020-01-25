/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...

    private final XboxController mXboxControllerPrimary = new XboxController(Constants.kXboxControllerUsbChannelPrimary);
    private final XboxController mXboxControllerSecondary = new XboxController(Constants.kXboxControllerUsbChannelSecondary);

    private final AbsoluteFieldPositionDeviceSubsystem mAbsoluteFieldPositionDeviceSubsystem = new AbsoluteFieldPositionDeviceSubsystem();
    private final BallLoadingDockHighHarvestorSubsystem mBallLoadingDockHighHarvestorSubsystem = new BallLoadingDockHighHarvestorSubsystem();
    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private final CameraDriverAftSubsystem mCameraDriverAftSubsystem = new CameraDriverAftSubsystem();
    private final CameraDriverForwardSubsystem mCameraDriverForwardSubsystem = new CameraDriverForwardSubsystem();
    private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
    private final CollisionAvoidanceSubsystem mCollisionAvoidanceSubsystem = new CollisionAvoidanceSubsystem();
    private final WheelControllerSubsystem mColorWheelControllerSubsystem = new WheelControllerSubsystem();
    private final CompressedAirSubsystem mCompressedAirSubsystem = new CompressedAirSubsystem();
    private final DrivebaseSubsystem mDrivebaseSubsystem = new DrivebaseSubsystem();
    private final GyroscopeSubsystem mGyroscopeSubsystem = new GyroscopeSubsystem();
    private final PowerSubsystem mPowerSubsystem = new PowerSubsystem();
    private final ShooterSubsystem mShooterSubsystem =  new ShooterSubsystem();
    private final VisionAlignmentSubsystem mVisionAlignmentSubsystem = new VisionAlignmentSubsystem();

    private final AutonomousCommand mAutonomousCommand = new AutonomousCommand();

    //TODO:
    /*
    private final ExtendClimbShaftUpwardCommand mExtendClimbShaftUpwardCommand = new ExtendClimbShaftUpwardCommand(mClimberSubsystem);
    private final RetractClimbShaftDownwardCommand mRetractClimbShaftDownwardCommand = new RetractClimbShaftDownwardCommand(mClimberSubsystem);
    private final ToggleDrivebaseGearingCommand mToggleDrivebaseGearingCommand = new ToggleDrivebaseGearingCommand(mDrivebaseSubsystem, mXboxControllerPrimary);
    private final ToggleShooterElevation mToggleShooterElevation = new ToggleShooterElevation(mShooterSubsystem, mXboxControllerSecondary);
    */
    // The robot's subsystems and commands are defined here...
    private JoystickButton buttonExtendClimbShaftUpward;
    private JoystickButton buttonRetractClimbShaftDownward;
    private JoystickButton buttonToggleDrivebaseGearing;
    private JoystickButton buttonToggleShooterElevation;
    private JoystickButton buttonStartWheelPositionControl;
    private JoystickButton buttonStartWheelColorControl;
    private JoystickButton buttonTogglePrecisionDriving;
    private JoystickButton buttonToggleIntakeExtender;
    private JoystickButton buttonIngestIntake;
    private JoystickButton buttonEjectIntake;


    /**
    * The container for the robot.  Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {
        configureButtonBindings();
        //CommandScheduler.getInstance().setDefaultCommand(mDrivebaseSubsystem, new ManualDriveCommand(mDrivebaseSubsystem, mXboxControllerPrimary));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //PRIMARY CONTROLLER
        buttonExtendClimbShaftUpward = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kX.value);
        buttonExtendClimbShaftUpward.whenPressed(new ExtendClimbShaftUpwardCommand(mClimberSubsystem));

        buttonRetractClimbShaftDownward = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kY.value);
        buttonRetractClimbShaftDownward.whenPressed(new RetractClimbShaftDownwardCommand(mClimberSubsystem));

        buttonToggleDrivebaseGearing = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kA.value);
        buttonToggleDrivebaseGearing.whenPressed(new ToggleDrivebaseGearingCommand(mDrivebaseSubsystem, mXboxControllerPrimary));

        buttonTogglePrecisionDriving = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kB.value);
        buttonTogglePrecisionDriving.whenPressed(new TogglePrecisionDrivingCommand(mDrivebaseSubsystem));

        
        //SECONDARY CONTROLLER
        buttonToggleShooterElevation = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kY.value);
        buttonToggleShooterElevation.whenPressed(new ToggleShooterElevationCommand(mShooterSubsystem, mXboxControllerSecondary));

        buttonStartWheelPositionControl = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kX.value);
        buttonStartWheelPositionControl.whenPressed(new WheelPositionControlCommand(mColorWheelControllerSubsystem));

        buttonStartWheelColorControl = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kB.value);
        buttonStartWheelColorControl.whenPressed(new WheelColorControlCommand(mColorWheelControllerSubsystem));

        buttonToggleIntakeExtender = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kA.value);
        buttonToggleIntakeExtender.whenPressed(new ToggleIntakeExtenderCommand(mIntakeSubsystem));

        buttonIngestIntake = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kBumperRight.value);
        buttonIngestIntake.whileHeld(new RunIntakeIngestCommand(mIntakeSubsystem));

        buttonEjectIntake = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kBumperLeft.value);
        buttonEjectIntake.whileHeld(new RunIntakeEjectCommand(mIntakeSubsystem));
        
        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Extend Climb Shaft Upward", new ExtendClimbShaftUpwardCommand(mClimberSubsystem));
        SmartDashboard.putData("Retract Climb Shaft Downward", new RetractClimbShaftDownwardCommand(mClimberSubsystem));
        //SmartDashboard.putData("Toggle Drivebase Gearing", new ToggleDrivebaseGearingCommand(mDrivebaseSubsystem, mXboxControllerPrimary));

    }

    public XboxController getXboxControllerPrimary() {
        return mXboxControllerPrimary;
    }

    public XboxController getXboxControllerSecondary() {
        return mXboxControllerSecondary;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
         return mAutonomousCommand;
     }
}


package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.ToggleCollisionAvoidanceCommand;
import frc.robot.commands.drive.ToggleDrivebaseGearingCommand;
import frc.robot.commands.drive.TogglePrecisionDrivingCommand;
import frc.robot.commands.superstructure.SetArmedModeCommand;
import frc.robot.commands.superstructure.SetIdleModeCommand;
import frc.robot.commands.superstructure.intake.EjectIntakeCommand;
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
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.WheelControllerSubsystem;

public class RobotContainer {

    private final XboxController mXboxControllerPrimary = new XboxController(Constants.kControllerPrimaryId);
    private final XboxController mXboxControllerSecondary = new XboxController(Constants.kControllerSecondaryId);

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

    private JoystickButton bToggleDriveGear;
    private JoystickButton bToggleCollisionAvoidance;
    private JoystickButton bStartWheelPositionControl;
    private JoystickButton bStartWheelColorTargeting;
    private JoystickButton bTogglePrecisionDrive;
    private JoystickButton bEjectIntake;
    private JoystickButton bStartShooter;
    private JoystickButton bShooterAuthorized;

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

        bToggleCollisionAvoidance = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kBack.value);
        bToggleCollisionAvoidance.whenPressed(new ToggleCollisionAvoidanceCommand(mCollisionAvoidanceSubsystem, mXboxControllerPrimary));
        
        bEjectIntake = new JoystickButton(mXboxControllerPrimary, XboxController.Button.kBumperLeft.value);
        bEjectIntake.whileHeld(new EjectIntakeCommand(mIntake));
        
        //SECONDARY CONTROLLER
        bStartWheelPositionControl = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kX.value);
        bStartWheelPositionControl.whenPressed(new WheelPositionControlCommand(mColorWheelController));

        bStartWheelColorTargeting = new JoystickButton(mXboxControllerSecondary, XboxController.Button.kB.value);
        bStartWheelColorTargeting.whenPressed(new WheelColorControlCommand(mColorWheelController));

        bStartShooter = new JoystickButton(mXboxControllerSecondary, XboxController.Axis.kLeftTrigger.value);
        bStartShooter
        .whenReleased(new SetIdleModeCommand(mSuperstructure))
        .whileActiveOnce(new SetArmedModeCommand(mSuperstructure));

        bShooterAuthorized = new JoystickButton(mXboxControllerSecondary, XboxController.Axis.kRightTrigger.value);
        bShooterAuthorized
        .whenReleased(new DeauthorizeShotCommand(mSuperstructure))
        .whileActiveOnce(new AuthorizeShotCommand(mSuperstructure));

        //TODO: Find how to use Joysticks like this
        //jTurret = new JoystickButton(mXboxControllerSecondary, XboxController.Axis.kLeftX.value);
        //jTurret.whileActiveContinuous(new DriveTurretCommand(mTurret, mXboxControllerSecondary));

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


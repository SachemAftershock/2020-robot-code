package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.groups.StartAutoFireSequence;
import frc.robot.Limelight;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.SuperstructureConstants.TurretConstants;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.groups.StartArmedSequence;
import frc.robot.commands.groups.StartFeedSequence;
import frc.robot.commands.groups.StartIdleSequence;

/**
 * Superstucture consisting of {@link ShooterSubsystem}, {@link TurretSubsystem}, {@link StorageSubsystem}, & {@link IntakeSubsystem}
 * 
 * @author Shreyas Prasad
 */
public class SuperstructureSubsystem extends SubsystemBase implements SubsystemInterface {

    private static SuperstructureSubsystem mInstance;

    private final ShooterSubsystem mShooter;
    //private final TurretSubsystem mTurret; //TODO: Change when Turret Implemented
    private final IntakeSubsystem mIntake;
    private final StorageSubsystem mStorage;

    private final DriveSubsystem mDrive;

    private final LimelightManagerSubsystem mLimelightManager;
    private final Limelight mShooterLimelight;

    private SuperstructureMode mSystemMode;
    private ShootingMode mShootingMode;

    private boolean mAuthorizedToShoot, mNewAutomaticMagazine;

    /**
     * Constructor for SuperstructureSubsystem Class
     */
    private SuperstructureSubsystem() {
        mShooter = ShooterSubsystem.getInstance();
        //mTurret = TurretSubsystem.getInstance(); //TODO: Change when Turret Implemented
        mIntake = IntakeSubsystem.getInstance();
        mStorage = StorageSubsystem.getInstance();

        mDrive = DriveSubsystem.getInstance();

        mLimelightManager = LimelightManagerSubsystem.getInstance();
        mShooterLimelight = mLimelightManager.getShooterLimelight();

        mSystemMode = SuperstructureMode.eIdle;
        mShootingMode = ShootingMode.eSemiAuto; //TODO: Cannot run Automatic Firing as the Feeder wheel has been taken out, change when fixed

        mAuthorizedToShoot = false;
        mNewAutomaticMagazine = true;
    }

    @Override
    public void init() {
    }

    /**
     * Superstructure State Machine Processor
     */
    @Override
    public void periodic() {
        switch(mSystemMode) {
            case eFeed:
                if (mStorage.isNewBallInChamber()) {
                    //setMode(SuperstructureMode.eArmed); //TODO: Change when Turret Implemented
                } else {
                    if(mStorage.isNewBallInIntake()) {
                        mStorage.runBelt();
                    } 
                    if(mStorage.isNewBallInMagazineEntry()) {
                        mStorage.stopBelt();
                    }
                }
                break;
            case eArmed:
                //Copied bc I didnt want to mess with the original block of code
                mShooter.reachCalculatedTargetRPM();
                final boolean atTargetRPM = mShooter.isAtTargetRPM();
                if(isAimedAtTarget()) {
                    if(atTargetRPM) {
                        if(mShootingMode == ShootingMode.eAuto) {
                            if(mNewAutomaticMagazine) {
                                CommandScheduler.getInstance().schedule(new StartAutoFireSequence(mShooter, mStorage));
                                mNewAutomaticMagazine = false;
                            } else if(mStorage.isEmpty()) {
                                setMode(SuperstructureMode.eIdle);
                                break;
                            }
                        } else if(mShootingMode == ShootingMode.eSemiAuto) {
                            if(!mStorage.isChamberLoaded()) {
                                mStorage.openChamberValve();
                                mStorage.runBelt();
                            } else {
                                mStorage.closeChamberValve();
                                mStorage.stopBelt();

                                if(atTargetRPM) {
                                    //mShooter.startFeeder();
                                    mShooter.loadBall();
                                } else {
                                // mShooter.stopFeeder();
                                mShooter.openBallLoader();
                                }
                            }
                        }
                    } else {
                        DriverStation.reportError("ERROR: SHOOTING MODE NOT FOUND", false);
                    }
                } else {
                    if(!mDrive.isAutoRotateRunning()) {
                        double tx = mShooterLimelight.getTx();
                        double theta = mDrive.getHeading() + (tx > 0 ? -tx : tx); // Error on limelight(tx) would be in opp direction(sign) as the direction(sign) of the drivebase
                        CommandScheduler.getInstance().schedule(new RotateDriveCommand(mDrive, theta));
                    }
                }
                /* //TODO: Change when Turret Implemented
                final boolean aimedAtTarget = mTurret.isAimedAtTarget();
                final boolean atTargetRPM = mShooter.isAtTargetRPM();
                mShooter.reachCalculatedTargetRPM();
                if(aimedAtTarget && atTargetRPM) {
                }
                if(mAuthorizedToShoot && aimedAtTarget) {
                    if(mShootingMode == ShootingMode.eAuto) {
                        if(mIsNewAutomaticMagazine) {
                            CommandScheduler.getInstance().schedule(new StartAutoFireSequence(mShooter, mStorage));
                            mIsNewAutomaticMagazine = false;
                        } else if(mStorage.isEmpty()) {
                            setMode(SuperstructureMode.eIdle);
                            break;
                        }
                    } else if(mShootingMode == ShootingMode.eSemiAuto) {
                        if(!mStorage.isChamberLoaded()) {
                            mStorage.openChamberValve();
                            mStorage.runBelt();
                        } else {
                            mStorage.closeChamberValve();
                            mStorage.stopBelt();

                            if(atTargetRPM) {
                                //mShooter.startFeeder();
                                mShooter.loadBall();
                            } else {
                               // mShooter.stopFeeder();
                               mShooter.openBallLoader();
                            }
                        }
                    } else {
                        DriverStation.reportError("ERROR: SHOOTING MODE NOT FOUND", false);
                    }
                }
                */
                break;
            case eIdle:
            default:
        }
    }

    /**
     * Run Initialization Commands for the Selected Mode
     * 
     * @param mode SuperstructureMode to change to
     */
    public void setMode(SuperstructureMode mode) {
        switch(mode) {
            case eFeed:
                CommandScheduler.getInstance().schedule(new StartFeedSequence(mIntake, mStorage));
                break;
            case eArmed:
                CommandScheduler.getInstance().schedule(new StartArmedSequence(mIntake, mShooter, mStorage));
                mNewAutomaticMagazine = true;
                break;
            case eIdle:
                CommandScheduler.getInstance().schedule(new StartIdleSequence(mIntake, mStorage, mShooter));
                break;
            default:
                mSystemMode = SuperstructureMode.eIdle;
                DriverStation.reportError("ERROR: SUPERSTRUCTURE MODE NOT FOUND", false);
                return;
        }
        mSystemMode = mode;
    }

    /**
     * Operator authorization for the Robot to fire at will
     */
    public void authorizeShot() {
        mAuthorizedToShoot = true;
    }

    /**
     * Operator deauthorization, the Robot cannot shoot
     */
    public void deauthorizeShot() {
        mAuthorizedToShoot = false;
        mNewAutomaticMagazine = true;
    }

    /**
     * If the shot has been authorized by the operator or not
     * 
     * @return <i> true </i> if the operator has authorized the Robot to shoot at will; <i> false </i> otherwise
     */
    public boolean isShotAuthorized() {
        return mAuthorizedToShoot;
    }

    /**
     * Gets current Superstructure System Mode
     * 
     * @return the mode of the Superstructure
     */
    public SuperstructureMode getCurrentMode() {
        return mSystemMode;
    }

    /**
     * System Mode of the Superstructure
     * <ul>
     * <li> Feed Mode: When Intake is engaged, packs Power Cells front to back to optimize number of balls stored
     * 
     * <li> Armed Mode: Preparing to Shoot
     * 
     * <li> Idle Mode: Default Mode, does nothing
     */
    public enum SuperstructureMode implements Sendable {
        eFeed(), eArmed(), eIdle();

        private SuperstructureMode() {}

        @Override
        public void initSendable(SendableBuilder builder) {}
    }

    /**
     * Enum to hold Shoting modes
     * <ul>
     * <li> Automatic Fire
     * <li> Semi-Automatic (Single Fire)
     */
    public enum ShootingMode implements Sendable {
        eSemiAuto(), eAuto();

        private ShootingMode() {}

        @Override
        public void initSendable(SendableBuilder builder) {}
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putData("Current System Mode", mSystemMode);
        SmartDashboard.putData("Current Shooting Mode", mShootingMode);
        SmartDashboard.putBoolean("Is Authorized to Shoot", mAuthorizedToShoot);
    }

    @Override
    public void runTest() {
        // TODO Auto-generated method stub
    }

    /**
     * @return SuperstructureSubsystem Singleton Instance
     */
    public synchronized static SuperstructureSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }

    //TODO: Change when Turret Implemented
    /**
     * Is Turret Aimed at Target AND does the ball have clearance with this angle
     * 
     * @return whether to take the shot
     */
    public synchronized boolean isAimedAtTarget() {
        double tx = mShooterLimelight.getTx();
        return Math.abs(tx) < SuperstructureConstants.kDrivebaseTargetingEpsilon 
                && TurretConstants.kTargetWidth[TurretSubsystem.ShootingTarget.eHighTarget.ordinal()] * Math.cos(Math.abs(DriveSubsystem.getInstance().getHeading() + tx)) - TurretConstants.kPowerCellClearance > TurretConstants.kPowerCellDiameterInches;
    }
}
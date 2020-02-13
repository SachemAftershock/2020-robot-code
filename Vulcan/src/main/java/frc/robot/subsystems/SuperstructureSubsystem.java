package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.groups.StartAutoFireSequence;
import frc.robot.commands.groups.StartArmedSequence;
import frc.robot.commands.groups.StartFeedSequence;
import frc.robot.commands.groups.StartIdleSequence;

/**
 * Superstucture consisting of Shooter, Turret, Intake, & Storage
 * @author Shreyas Prasad
 */
public class SuperstructureSubsystem extends SubsystemBase implements SubsystemInterface {

    private static SuperstructureSubsystem mInstance;

    private final ShooterSubsystem mShooter;
    private final TurretSubsystem mTurret;
    private final IntakeSubsystem mIntake;
    private final StorageSubsystem mStorage;

    private SuperstructureMode mSystemMode;
    private ShootingMode mShootingMode;

    private boolean mAuthorizedToShoot, mIsNewAutomaticMagazine;

    private SuperstructureSubsystem() {
        mShooter = ShooterSubsystem.getInstance();
        mTurret = TurretSubsystem.getInstance();
        mIntake = IntakeSubsystem.getInstance();
        mStorage = StorageSubsystem.getInstance();

        mSystemMode = SuperstructureMode.eIdle;
        mShootingMode = ShootingMode.eAuto;

        mAuthorizedToShoot = false;
        mIsNewAutomaticMagazine = true;
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
                    setMode(SuperstructureMode.eArmed);
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
                break;
            case eIdle:
            default:
        }
    }

    /**
     * Run Initialization Commands for the Selected Mode
     * @param mode SuperstructureMode to change to
     */
    public void setMode(SuperstructureMode mode) {
        switch(mode) {
            case eFeed:
                CommandScheduler.getInstance().schedule(new StartFeedSequence(mIntake, mStorage));
                break;
            case eArmed:
                CommandScheduler.getInstance().schedule(new StartArmedSequence(mIntake, mShooter, mStorage));
                mIsNewAutomaticMagazine = true;
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

    public void authorizeShot() {
        mAuthorizedToShoot = true;
    }
    public void deauthorizeShot() {
        mAuthorizedToShoot = false;
        mIsNewAutomaticMagazine = true;
    }

    public boolean isShotAuthorized() {
        return mAuthorizedToShoot;
    }

    public SuperstructureMode getCurrentMode() {
        return mSystemMode;
    }

    public enum SuperstructureMode implements Sendable {
        eFeed(), eArmed(), eIdle();

        private SuperstructureMode() {}

        @Override
        public void initSendable(SendableBuilder builder) {}
    }

    public enum ShootingMode implements Sendable {
        eSemiAuto(), eAuto();

        private ShootingMode() {}

        @Override
        public void initSendable(SendableBuilder builder) {}
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData("Current System Mode", mSystemMode);
        SmartDashboard.putData("Current Shooting Mode", mShootingMode);
        SmartDashboard.putBoolean("Is Authorized to Shoot", mAuthorizedToShoot);
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
}
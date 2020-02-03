package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.groups.StartAutoFireSequence;
import frc.robot.commands.groups.StartArmedSequence;
import frc.robot.commands.groups.StartFeedSequence;
import frc.robot.commands.groups.StartIdleSequence;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem mInstance;

    private final ShooterSubsystem mShooter;
    private final TurretSubsystem mTurret;
    private final IntakeSubsystem mIntake;
    private final StorageSubsystem mStorage;

    private SuperstructureMode mCurrentMode;
    private ShootingMode mShootingMode;

    private boolean mAuthorizedToShoot, mIsNewAutomaticMagazine;

    public SuperstructureSubsystem() {
        mShooter = ShooterSubsystem.getInstance();
        mTurret = TurretSubsystem.getInstance();
        mIntake = IntakeSubsystem.getInstance();
        mStorage = StorageSubsystem.getInstance();

        mCurrentMode = SuperstructureMode.eIdle;
        mShootingMode = ShootingMode.eAuto;

        mAuthorizedToShoot = false;
        mIsNewAutomaticMagazine = true;
    }

    @Override
    public void periodic() {
        switch(mCurrentMode) {
            case eFeed:
                if (mStorage.isNewBallInChamber()) {
                    setMode(SuperstructureMode.eArmed);
                }
                if(mStorage.isNewBallInIntake()) {
                    mStorage.runBelt();
                }
                if(mStorage.isNewBallInMagazineEntry()) {
                    mStorage.stopBelt();
                }
                break;
            case eArmed:
                mShooter.reachCalculatedTargetRPM();
                //TODO: Implement some way to find out whether it's worth it to take the shot or not
                if(mAuthorizedToShoot && mTurret.isAimedAtTarget()) {
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
                        }

                        if(mShooter.isAtTargetRPM()) {
                            mShooter.startFeeder();
                        } else {
                            mShooter.stopFeeder();
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
                DriverStation.reportError("ERROR: SUPERSTRUCTURE MODE NOT FOUND", false);
        }
        mCurrentMode = mode;
    }

    public void authorizeShot() {
        mAuthorizedToShoot = true;
    }
    public void deauthorizeShot() {
        mAuthorizedToShoot = false;
        mIsNewAutomaticMagazine = true;
    }

    public SuperstructureMode getCurrentMode() {
        return mCurrentMode;
    }

    public enum SuperstructureMode {
        eFeed, eArmed, eIdle
    }

    public enum ShootingMode {
        eSemiAuto, eAuto
    }

    public synchronized static SuperstructureSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }
}
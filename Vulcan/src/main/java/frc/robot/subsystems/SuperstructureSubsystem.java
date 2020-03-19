package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AftershockSubsystem;
import frc.lib.LatchedBoolean;
import frc.lib.Limelight;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.groups.StartArmedSequence;
import frc.robot.commands.groups.StartFeedSequence;
import frc.robot.commands.groups.StartIdleSequence;
import frc.robot.subsystems.StorageSubsystem.BallPosition;

/**
 * Superstucture consisting of {@link ShooterSubsystem}, {@link TurretSubsystem}, {@link StorageSubsystem}, & {@link IntakeSubsystem}
 * 
 * @author Shreyas Prasad
 */
public class SuperstructureSubsystem extends AftershockSubsystem {

    private static SuperstructureSubsystem mInstance;

    private final ShooterSubsystem mShooter;
    //private final TurretSubsystem mTurret;
    private final IntakeSubsystem mIntake;
    private final StorageSubsystem mStorage;

    private final DriveSubsystem mDrive;

    private final LimelightManagerSubsystem mLimelightManager;
    private final Limelight mShooterLimelight;

    private SuperstructureMode mSystemMode;
    private ShootingMode mShootingMode;

    private boolean mAuthorizedToShoot;

    private BallPosition mBallPosition;

    private SemiAutoStage mSemiAutoStage;

    private LatchedBoolean mNewlyPressedAuthorizedToShoot = new LatchedBoolean();

    /**
     * Constructor for SuperstructureSubsystem Class
     */
    private SuperstructureSubsystem() {
        super();
        setName("Superstructure Subsystem");

        mShooter = ShooterSubsystem.getInstance();
        //mTurret = TurretSubsystem.getInstance();
        mIntake = IntakeSubsystem.getInstance();
        mStorage = StorageSubsystem.getInstance();

        mDrive = DriveSubsystem.getInstance();

        mLimelightManager = LimelightManagerSubsystem.getInstance();
        mShooterLimelight = mLimelightManager.getShooterLimelight();

        mSystemMode = SuperstructureMode.eIdle;
        mShootingMode = ShootingMode.eFullAuto;
        
        mAuthorizedToShoot = false;
    }

    @Override
    public void initialize() {
        setMode(SuperstructureMode.eIdle);
    }

    /**
     * Superstructure State Machine Processor
     */
    @Override
    public void periodic() {
        if(mStorage.isPositionChamber()) { //TODO: Test if top "if" statement works
            mBallPosition = BallPosition.ePositionChamber;
        } else if(mStorage.isPositionC()) {
            mBallPosition = BallPosition.ePositionC;
        } else if(mStorage.isPositionB()) {
            mBallPosition = BallPosition.ePositionB;
        } else if(mStorage.isPositionA()) {
            mBallPosition = BallPosition.ePositionA;
        } else if(mStorage.isEmpty()) {
            mBallPosition = BallPosition.eNone;
        } else {
            mBallPosition = BallPosition.eNone;
            DriverStation.reportWarning("WARNING: BALL POSITION NOT FOUND", false);
        }

        switch(mSystemMode) {
            case eFeed:
                if(mBallPosition == BallPosition.ePositionChamber) {
                    setMode(SuperstructureMode.eIdle);
                } else if(mBallPosition == BallPosition.ePositionC || mBallPosition == BallPosition.ePositionB || mBallPosition == BallPosition.ePositionA) {
                    if(mStorage.isBallInIntake()) {
                        mStorage.runBelt();
                    } else {
                        mStorage.stopBelt();
                    }
                } else if(mBallPosition == BallPosition.eNone) {
                    if(mStorage.isBallInIntake()) {
                        mStorage.runBelt();
                    } else {
                        mStorage.stopBelt();
                    }
                }
                break;

            case eArmed:
                mShooter.reachCalculatedTargetRPM();
                final boolean atTargetRPM = mShooter.isAtTargetRPM();
                if(mBallPosition != BallPosition.eNone && !mStorage.isBallInChamber()) {
                    mStorage.runBelt();
                } else {
                    mStorage.stopBelt();
                }
                if(isAimedAtTarget()) {
                    if(atTargetRPM) {
                        if(mShootingMode == ShootingMode.eFullAuto && mAuthorizedToShoot) {
                            if(mBallPosition != BallPosition.eNone || mStorage.isBallInIntake()) {
                            	mStorage.runBelt();
                            } else {
                                setMode(SuperstructureMode.eIdle);
                            }
                        } else if(mShootingMode == ShootingMode.eSemiAuto) {
                            processSemiAutoFireSequence();
                        }
                    } else {
                        //mStorage.stopBelt();
                    }
                } else {
                    if(!mDrive.isAutoRotateRunning()) {
                        //double theta = mDrive.getHeading() - mShooterLimelight.getTx(); //TODO: Check if works
                        //CommandScheduler.getInstance().schedule(new RotateDriveCommand(mDrive, theta));
                    }
                }
                break;
                
            case eIdle:
                if(mShooterLimelight.isTarget()) { //TODO: Test
                    //mShooter.setTargetRPM(ShooterConstants.kDistanceInToRpmLUT[ShooterConstants.kMinRPMCellIndex][ShooterConstants.kRPMIndex]);
                } else {
                    //mShooter.stopShooter();
                }
                break;
            default:
        }
    }

    public void processSemiAutoFireSequence() {
        if(mNewlyPressedAuthorizedToShoot.update(mAuthorizedToShoot)) {
            mSemiAutoStage = SemiAutoStage.eLoadBallChamber;
        }
        switch(mSemiAutoStage) {
            case eLoadBallChamber:
                if(mBallPosition != BallPosition.ePositionChamber) {
                    mStorage.runBelt();
                }
                mSemiAutoStage = SemiAutoStage.eFireBall;
                break;
            case eFireBall:
                if(mBallPosition != BallPosition.ePositionChamber) {
                    mStorage.stopBelt();
                    if(!mStorage.isEmpty()) {
                        mSemiAutoStage = SemiAutoStage.eLoadBallChamber;
                    } else {
                        mSemiAutoStage = SemiAutoStage.eEndSequence;
                    }
                }
                break;
            case eEndSequence:
                setMode(SuperstructureMode.eIdle);
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
     * Is Drivebase Aimed at Target AND does the ball have clearance with this angle
     * 
     * @return whether to take the shot
     */
    public synchronized boolean isAimedAtTarget() {
        double tx = mShooterLimelight.getTx();
        return Math.abs(Math.abs(tx) - 3.5) < SuperstructureConstants.kDrivebaseTargetingEpsilon; 
                //&& TurretConstants.kHighTargetWidthInches * Math.cos(Math.abs(DriveSubsystem.getInstance().getHeading() + tx)) - TurretConstants.kPowerCellClearance > TurretConstants.kPowerCellDiameterInches;
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
    public enum SuperstructureMode {
        eFeed(), eArmed(), eIdle();

        private SuperstructureMode() {}

        @Override
        public String toString() {
            return this.name();
        }
    }

    /**
     * Enum to hold Shoting modes
     * <ul>
     * <li> Automatic Fire
     * <li> Semi-Automatic (Single Fire)
     */
    public enum ShootingMode {
        eSemiAuto(), eFullAuto();

        private ShootingMode() {}

        @Override
        public String toString() {
            return this.name();
        }
    }

    private enum SemiAutoStage {
        eLoadBallChamber, eFireBall, eEndSequence;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putString("Current System Mode", mSystemMode.toString());
        SmartDashboard.putString("Current Shooting Mode", mShootingMode.toString());
        SmartDashboard.putString("Current Ball Pos", mBallPosition.toString());
        SmartDashboard.putBoolean("Is Authorized to Shoot", mAuthorizedToShoot);
        SmartDashboard.putBoolean("Is Aimed At Target", isAimedAtTarget());
    }

    @Override
    public boolean checkSystem() {
		return true;
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
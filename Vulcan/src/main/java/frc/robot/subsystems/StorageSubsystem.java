package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Lidar;
import frc.robot.Constants.SuperstructureConstants.StorageConstants;

/**
 * Power Cell Storage Subsystem
 * 
 * @author Shreyas Prasad
 */
public class StorageSubsystem extends SubsystemBase implements SubsystemInterface {

    private static StorageSubsystem mInstance;

    private final WPI_TalonSRX mBeltDriver;
    private final WPI_VictorSPX mLowerBeltDriver;
    //private final DigitalInput mChamberBallDetector, mPreChamberBallDetector, mIntakeBallDetector, mEntryBallDetector;
    private final Lidar mLidar;

    //private LatchedBoolean mNewBallInChamber, mNewBallInIntake, mNewBallEnteredMagazine;

    /**
     * Constructor for StorageSubsystem Class
     */
    private StorageSubsystem() {
        mLowerBeltDriver = new WPI_VictorSPX(StorageConstants.kBottomBeltDriverMotorId);
        mBeltDriver = new WPI_TalonSRX(StorageConstants.kBeltDriverMotorId);
        mBeltDriver.setNeutralMode(NeutralMode.Brake);

        /*
        mChamberBallDetector = new DigitalInput(StorageConstants.kChamberDetectorId);
        mPreChamberBallDetector = new DigitalInput(StorageConstants.kPreChamberDetectorId);
        mIntakeBallDetector = new DigitalInput(StorageConstants.kIntakeDetectorId);
        mEntryBallDetector = new DigitalInput(StorageConstants.kEntryDetectorId);
        */
        mLidar = new Lidar(new DigitalInput(StorageConstants.kLidarId));

        //mNewBallInChamber = new LatchedBoolean();
        //mNewBallInIntake = new LatchedBoolean();
        //mNewBallEnteredMagazine = new LatchedBoolean();
    }

    @Override
    public void init() {
    }

    /**
     * Gets if a New Power Cell has entered the Chamber
     * 
     * @return <i> true </i> if a Power Cell has entered the Chamber; <i> false </i> otherwise
     */
    public boolean isNewBallInChamber() {
        return true;//mNewBallInChamber.update(isChamberLoaded());
    }

    /**
     * Gets if a New Power Cell has entered the Intake
     * 
     * @return <i> true </i> if a Power Cell has entered the Intake; <i> false </i> otherwise
     */
    public boolean isNewBallInIntake() {
        return true;//mNewBallInIntake.update(isBallCaughtIntake());
    }

    /**
     * Gets if a New Power Cell has entered the Magazine
     * 
     * @return <i> true </i> if a Power Cell has entered the Magazine; <i> false </i> otherwise
     */
    public boolean isNewBallInMagazineEntry() {
       return true;//mNewBallEnteredMagazine.update(isBallEnteredMagazine());
    }

    /**
     * Lidar Checks if any Power Cells are in the storage
     * 
     * @return <i> true </i> if LIDAR detects a ball in the storage; <i> false </i> otherwise
     */
    public boolean isEmpty() {
        return mLidar.getDistanceIn() < StorageConstants.kMaxEmptyStorageDistanceInches;
    }

    /**
     * Checks if Power Cell is loaded in Chamber
     * 
     * @return <i> true </i> if a Power Cell is inside the Chamber; <i> false </i> otherwise
     */
    public boolean isChamberLoaded() {
        return false;//mChamberBallDetector.get();
    }

    /**
     * Checks if Power Cell is at the back of the magazine, ready to be loaded into the chamber
     * 
     * @return <i> true </i> if a Power Cell is at the back of the magazine; <i> false </i> otherwise
     */
     public boolean isBackMagazineLoaded() {
        return false;//mPreChamberBallDetector.get();
    }

    /**
     * Checks if Power Cell has entered the Magazine
     * 
     * @return <i> true </i> if a Power Cell has entered the magazine; <i> false </i> otherwise
     */
    public boolean isBallEnteredMagazine() {
        return false;//mEntryBallDetector.get();
    }

    /**
     * Checks if Power Cell is detected by Intake Beam Breaker
     * 
     * @return <i> true </i> if a Power Cell is caught in the intake; <i> false </i> otherwise
     */
    public boolean isBallCaughtIntake() {
        return false;//mIntakeBallDetector.get();
    }

    /**
     * Runs storage belt at constant speed
     */
    public void runBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, StorageConstants.kBeltSpeed);
        mLowerBeltDriver.set(ControlMode.PercentOutput, StorageConstants.kBeltSpeed);
    }

    /**
     * Runs storage belt in the opposite direction at constant speed
     */
    public void reverseBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, -StorageConstants.kBeltSpeed);
        mLowerBeltDriver.set(ControlMode.PercentOutput, -StorageConstants.kBeltSpeed);
    }

    /**
     * Stops storage belt from running
     */
    public void stopBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, 0.0);
        mLowerBeltDriver.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * An iterative boolean latch.
     * <p>
     * Returns true once if and only if the value of newValue changes from false to true.
     */
    public class LatchedBoolean {
        private boolean mLast = false;

        public boolean update(boolean newValue) {
            boolean ret = false;
            if (newValue && !mLast) {
                ret = true;
            }
            mLast = newValue;
            return ret;
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        //SmartDashboard.putBoolean("Ball in Intake", isBallCaughtIntake());
        //SmartDashboard.putBoolean("Ball in Magazine Entry", isBallEnteredMagazine());
        //SmartDashboard.putBoolean("Ball before Chamber", isBackMagazineLoaded());
        //SmartDashboard.putBoolean("Chamber Loaded", isChamberLoaded());
        SmartDashboard.putBoolean("Belt Running", mBeltDriver.get() != 0);
        SmartDashboard.putNumber("Storage Lidar", mLidar.getDistanceIn());
        SmartDashboard.putBoolean("Is Storage Empty", isEmpty());
    }

    @Override
    public void runTest() {
        // TODO Auto-generated method stub
    }

    /**
     * @return StorageSubsystem Singleton Instance
     */
    public static StorageSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new StorageSubsystem();
        }
        return mInstance;
    }
}
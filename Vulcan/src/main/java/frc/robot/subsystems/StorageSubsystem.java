package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LIDAR;
import frc.robot.Constants.SuperstructureConstants.StorageConstants;

public class StorageSubsystem extends SubsystemBase implements SubsystemInterface {

    private static StorageSubsystem mInstance;

    private final TalonSRX mBeltDriver;
    private final DoubleSolenoid mBallValveA, mBallValveB;
    private final DigitalInput mChamberBallDetector, mPreChamberBallDetector, mIntakeBallDetector, mEntryBallDetector;
    private final LIDAR mLidar;

    private boolean mPrevChamberLoaded, mPrevIntakeBallDetected, mPrevMagazineEntryBallDetected;

    public StorageSubsystem() {
        mBeltDriver = new WPI_TalonSRX(StorageConstants.kBeltDriverMotorId);
        mBeltDriver.setNeutralMode(NeutralMode.Brake);

        mBallValveA = new DoubleSolenoid(Constants.kPcmAId, StorageConstants.kBallValveAForwardId, StorageConstants.kBallValveAReverseId);
        mBallValveB = new DoubleSolenoid(Constants.kPcmAId, StorageConstants.kBallValveBForwardId, StorageConstants.kBallValveBReverseId);

        mChamberBallDetector = new DigitalInput(StorageConstants.kChamberDetectorId);
        mPreChamberBallDetector = new DigitalInput(StorageConstants.kPreChamberDetectorId);
        mIntakeBallDetector = new DigitalInput(StorageConstants.kIntakeDetectorId);
        mEntryBallDetector = new DigitalInput(StorageConstants.kEntryDetectorId);

        mLidar = new LIDAR(new DigitalInput(StorageConstants.kLidarId));
    }

    @Override
    public void init() {
        mPrevChamberLoaded = isChamberLoaded();
        mPrevIntakeBallDetected = isBallCaughtIntake();
        mPrevMagazineEntryBallDetected = isBallEnteredMagazine();
    }

    public void closeChamberValve() {
        mBallValveA.set(Value.kForward);
        mBallValveB.set(Value.kForward);
    }

    public void openChamberValve() {
        mBallValveA.set(Value.kReverse);
        mBallValveB.set(Value.kReverse);
    }

    public boolean isNewBallInChamber() {
        if (isChamberLoaded() && !mPrevChamberLoaded) {
            mPrevChamberLoaded = true;
            return true;
        } else {
            if(!isChamberLoaded()) {
                mPrevChamberLoaded = false;
            }
            return false;
        }
    }

    public boolean isNewBallInIntake() {
        if (isBallCaughtIntake() && !mPrevIntakeBallDetected) {
            mPrevIntakeBallDetected = true;
            return true;
        } else {
            if(!isBallCaughtIntake()) {
                mPrevIntakeBallDetected = false;
            }
            return false;
        }
    }

    public boolean isNewBallInMagazineEntry() {
        if (isBallEnteredMagazine() && !mPrevMagazineEntryBallDetected) {
            mPrevMagazineEntryBallDetected = true;
            return true;
        } else {
            if(!isBallEnteredMagazine()) {
                mPrevMagazineEntryBallDetected = false;
            }
            return false;
        }
    }

    public boolean isEmpty() {
        return mLidar.getDistanceIn() < StorageConstants.kMaxEmptyStorageDistanceInches;
    }

    public boolean isChamberLoaded() {
        return mChamberBallDetector.get();
    }

    public boolean isBackMagazineLoaded() {
        return mPreChamberBallDetector.get();
    }

    public boolean isBallEnteredMagazine() {
        return mEntryBallDetector.get();
    }

    public boolean isBallCaughtIntake() {
        return mIntakeBallDetector.get();
    }

    public void runBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, StorageConstants.kBeltSpeed);
    }

    public void stopBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, 0.0);
    }

    public static StorageSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new StorageSubsystem();
        }
        return mInstance;
    }
}
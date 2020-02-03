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

public class StorageSubsystem extends SubsystemBase {

    private static StorageSubsystem mInstance;

    private final TalonSRX mBeltDriver;
    private final DoubleSolenoid mBallValveA, mBallValveB;
    private final DigitalInput mChamberBallDetector, mPreChamberBallDetector, mIntakeBallDetector, mEntryBallDetector;

    private final double kBeltSpeed = 0.3;

    private boolean mPrevChamberLoaded, mPrevFrontMagazineLoaded, mPrevIntakeBallDetected, mPrevMagazineEntryBallDetected;

    public StorageSubsystem() {
        mBeltDriver = new WPI_TalonSRX(Constants.kFeederMotorId);
        mBeltDriver.setNeutralMode(NeutralMode.Brake);

        mBallValveA = new DoubleSolenoid(Constants.kPcmAId, Constants.kBallValveAForwardId, Constants.kBallValveAReverseId);
        mBallValveB = new DoubleSolenoid(Constants.kPcmAId, Constants.kBallValveBForwardId, Constants.kBallValveBReverseId);

        mChamberBallDetector = new DigitalInput(Constants.kChamberDetectorId);
        mPreChamberBallDetector = new DigitalInput(Constants.kPreChamberDetectorId);
        mIntakeBallDetector = new DigitalInput(Constants.kIntakeDetectorId);
        mEntryBallDetector = new DigitalInput(Constants.kEntryDetectorId);

        mPrevChamberLoaded = true; //TODO: Should init to whatever it actually sees
        mPrevFrontMagazineLoaded = true;
        mPrevIntakeBallDetected = false;
        mPrevMagazineEntryBallDetected = false;
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

    public boolean isNewBallAtMagazineFront() {
        if (isFrontMagazineLoaded() && !mPrevFrontMagazineLoaded) {
            mPrevFrontMagazineLoaded = true;
            return true;
        } else {
            if(!isFrontMagazineLoaded()) {
                mPrevFrontMagazineLoaded = false;
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

    //TODO: Might need a better definition of "Empty"
    public boolean isEmpty() {
        return !isChamberLoaded() && !isFrontMagazineLoaded() && !isBallEnteredMagazine();
    }

    public boolean isChamberLoaded() {
        return mChamberBallDetector.get();
    }

    public boolean isFrontMagazineLoaded() {
        return mPreChamberBallDetector.get();
    }

    public boolean isBallEnteredMagazine() {
        return mEntryBallDetector.get();
    }

    public boolean isBallCaughtIntake() {
        return mIntakeBallDetector.get();
    }

    public void runBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, kBeltSpeed);
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
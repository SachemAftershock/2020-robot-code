package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lidar;
import frc.robot.Constants.SuperstructureConstants.StorageConstants;

/**
 * Power Cell Storage Subsystem
 * 
 * @author Shreyas Prasad
 */
public class StorageSubsystem extends SubsystemBase implements SubsystemInterface {

    private static StorageSubsystem mInstance;

    private final WPI_TalonSRX mBeltDriver, mLowerBeltDriver;
    private final DigitalOutput mIntakeBallEmitter, mChamberBallEmitter;
    private final DigitalInput mIntakeBallDetector, mChamberBallDetector;
    private final Lidar mLidar;

    private final MedianFilter mMedianFilter;

    /**
     * Constructor for StorageSubsystem Class
     */
    private StorageSubsystem() {
        mBeltDriver = new WPI_TalonSRX(StorageConstants.kBeltDriverMotorId);
        mBeltDriver.setNeutralMode(NeutralMode.Brake);
        mBeltDriver.setInverted(false);
        addChild("Belt Driver", mBeltDriver);

        mLowerBeltDriver = new WPI_TalonSRX(StorageConstants.kBottomBeltDriverMotorId);
        mLowerBeltDriver.setNeutralMode(NeutralMode.Brake);
        mBeltDriver.setInverted(false);
        addChild("Lower Belt Driver", mLowerBeltDriver);

        mIntakeBallEmitter = new DigitalOutput(StorageConstants.kIntakeEmitterId);
        mIntakeBallDetector = new DigitalInput(StorageConstants.kIntakeDetectorId);

        mChamberBallEmitter = new DigitalOutput(StorageConstants.kChamberEmitter);
        mChamberBallDetector = new DigitalInput(StorageConstants.kChamberDetectorId);
        
        mLidar = new Lidar(new DigitalInput(StorageConstants.kLidarId));

        mMedianFilter = new MedianFilter(StorageConstants.kMedianFilterSize);

        //mNewBallInChamber = new LatchedBoolean();
        //mNewBallInIntake = new LatchedBoolean();
        //mNewBallEnteredMagazine = new LatchedBoolean();
    }

    @Override
    public void init() {
        mIntakeBallEmitter.set(true);
        mChamberBallEmitter.set(true);
        mLowerBeltDriver.set(0.0);
        mBeltDriver.set(0.0);
    }

    @Override
    public void periodic() {
    }

    public boolean isBallInIntake() {
        return !mIntakeBallDetector.get();
    }

    public boolean isBallInChamber() {
        return !mChamberBallDetector.get();
    }

    public boolean isPositionA() {
        return getFilteredDistanceIn() <= BallPosition.ePositionA.getLeadingEdge();
    }

    public boolean isPositionB() {
        return getFilteredDistanceIn() <= BallPosition.ePositionB.getLeadingEdge();
    }

    public boolean isPositionC() {
        return getFilteredDistanceIn() <= BallPosition.ePositionC.getLeadingEdge();
    }

    public boolean isPositionChamber() {
        return getFilteredDistanceIn() <= BallPosition.ePositionChamber.getLeadingEdge();
    }

    /**
     * Lidar Checks if any Power Cells are in the storage
     * 
     * @return <i> true </i> if LIDAR detects a ball in the storage; <i> false </i> otherwise
     */
    public boolean isEmpty() {
        return getFilteredDistanceIn() >= BallPosition.eNone.getLeadingEdge();
    }

    /**
     * Runs storage belt at constant speed
     */
    public void runBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, -StorageConstants.kBeltSpeed);
        mLowerBeltDriver.set(ControlMode.PercentOutput, StorageConstants.kLowerBeltSpeed);
    }

    /**
     * Runs storage belt in the opposite direction at constant speed
     */
    public void reverseBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, StorageConstants.kBeltSpeed);
        mLowerBeltDriver.set(ControlMode.PercentOutput, -StorageConstants.kLowerBeltSpeed);
    }

    /**
     * Stops storage belt from running
     */
    public void stopBelt() {
        mBeltDriver.set(ControlMode.PercentOutput, 0.0);
        mLowerBeltDriver.set(ControlMode.PercentOutput, 0.0);
    }

    public double getFilteredDistanceIn() {
        return mMedianFilter.calculate(mLidar.getDistanceIn());
    }

    public enum BallPosition {
        eNone(StorageConstants.kBallInIntakeMaxIn), ePositionA(StorageConstants.kRangeToPositionALeadingEdgeIn),
        ePositionB(StorageConstants.kRangeToPositionBLeadingEdgeIn), ePositionC(StorageConstants.kRangeToPositionCLeadingEdgeIn),
        ePositionChamber(StorageConstants.kRangeToPositionChamberLeadingEdgeIn);

        private final double leadingEdge;

        private BallPosition(double leadingEdge) {
            this.leadingEdge = leadingEdge;
        }

        public double getLeadingEdge() {
            return this.leadingEdge;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        //SmartDashboard.putBoolean("Ball in Intake", isBallCaughtIntake());
        SmartDashboard.putBoolean("Belt Running", mBeltDriver.get() != 0);
        SmartDashboard.putNumber("Storage Lidar", getFilteredDistanceIn());
        SmartDashboard.putBoolean("Is Storage Empty", isEmpty());
        SmartDashboard.putBoolean("Is Ball in Intake", isBallInIntake());
        SmartDashboard.putBoolean("Is Ball in Chamber", isBallInChamber());
        SmartDashboard.putData(mBeltDriver);
        SmartDashboard.putData(mLowerBeltDriver);
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
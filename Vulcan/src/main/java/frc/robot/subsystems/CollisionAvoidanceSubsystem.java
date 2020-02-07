package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollisionAvoidanceConstants;

public class CollisionAvoidanceSubsystem extends SubsystemBase implements SubsystemInterface {

    private static CollisionAvoidanceSubsystem mInstance;

    private boolean mCollisionEnabled;
    private double mCurrentDistance;
    private double mSelectedStandoffSlowdown;

    private final MedianFilter mMedianFilter = new MedianFilter(CollisionAvoidanceConstants.kNumberOfDistanceSamples);
    private final AnalogInput mUltrasonic;


    public CollisionAvoidanceSubsystem() {
        mCollisionEnabled = true;
        mUltrasonic = new AnalogInput(CollisionAvoidanceConstants.kCollisionUltrasonicId);
        mSelectedStandoffSlowdown = CollisionAvoidanceConstants.kCollisionStandoffSlowdownInches;
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        mCurrentDistance = getUltrasonicDistanceInches();
    }

    public double getSlowdownScaleFactor() {
        if(!mCollisionEnabled) {
            return 1.0;
        } else if(getCurrentDistanceInches() <= mSelectedStandoffSlowdown) {
            return getCurrentDistanceInches() / mSelectedStandoffSlowdown;
        } else {
            return 1.0;
        }
    }

    public void setColorWheelStandoff() {
        mSelectedStandoffSlowdown = CollisionAvoidanceConstants.kColorWheelStandoffSlowdownInches;
    }
    public void setStandardStandoff() {
        mSelectedStandoffSlowdown = CollisionAvoidanceConstants.kCollisionStandoffSlowdownInches;
    }

    public void enableCollisionAvoidance() {
        mCollisionEnabled = true;
    }
    public void disableCollisionAvoidance() {
        mCollisionEnabled = false;
    }

    public boolean isCollisionAvoidanceEnabled() {
        return mCollisionEnabled;
    }

    public double getCurrentDistanceInches() {
        return mCurrentDistance;
    }

    private double getUltrasonicDistanceInches() {
        return mMedianFilter.calculate(mUltrasonic.getValue() * CollisionAvoidanceConstants.kUltrasonicValueToInches);
    }

    public synchronized static CollisionAvoidanceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CollisionAvoidanceSubsystem();
        }
        return mInstance;
    }
}


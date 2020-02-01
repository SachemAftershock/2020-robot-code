package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollisionAvoidanceSubsystem extends SubsystemBase {

    private static CollisionAvoidanceSubsystem mInstance;

    private boolean mCollisionEnabled;
    private double mCurrentDistance;
    private double mSelectedStandoffSlowdown;

    private final int kNumberOfDistanceSamples = 10;
    //TODO: Find good values for the below
    private final double kCollisionStandoffSlowdown = 18.0;
    private final double kColorWheelStandoffSlowdown = 12.0;


    private final MedianFilter mMedianFilter = new MedianFilter(kNumberOfDistanceSamples);
    private final AnalogInput mUltrasonic;


    public CollisionAvoidanceSubsystem() {
        mCollisionEnabled = true;
        mUltrasonic = new AnalogInput(Constants.kCollisionUltrasonicId);
        mSelectedStandoffSlowdown = kCollisionStandoffSlowdown;
    }

    @Override
    public void periodic() {
        mCurrentDistance = getUltrasonicDistance();
    }

    public double getSlowdownScaleFactor() {
        if(!mCollisionEnabled) {
            return 1.0;
        } else if(getCurrentDistance() <= mSelectedStandoffSlowdown) {
            return getCurrentDistance() / mSelectedStandoffSlowdown;
        } else {
            return 1.0;
        }
    }

    public void setColorWheelStandoff() {
        mSelectedStandoffSlowdown = kColorWheelStandoffSlowdown;
    }
    public void setStandardStandoff() {
        mSelectedStandoffSlowdown = kCollisionStandoffSlowdown;
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

    public double getCurrentDistance() {
        return mCurrentDistance;
    }

    private double getUltrasonicDistance() {
        return mMedianFilter.calculate(mUltrasonic.getValue() * Constants.kUltrasonicValueToInches);
    }

    public synchronized static CollisionAvoidanceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CollisionAvoidanceSubsystem();
        }
        return mInstance;
    }
}


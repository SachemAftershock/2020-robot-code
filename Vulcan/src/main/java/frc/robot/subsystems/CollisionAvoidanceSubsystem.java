package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollisionAvoidanceSubsystem extends SubsystemBase {

    private static CollisionAvoidanceSubsystem mInstance;

    private boolean mCollisionEnabled;
    private double mCurrentDistance;

    private final int kNumberOfDistanceSamples = 10;

    private final MedianFilter mMedianFilter = new MedianFilter(kNumberOfDistanceSamples);
    private final AnalogInput mUltrasonic;

    private final double kCollisionStandoffSlowdown = 18.0;
    private final double kCollisionStandoffStop = 6.0;

    public CollisionAvoidanceSubsystem() {
        mCollisionEnabled = true;
        mUltrasonic = new AnalogInput(Constants.kCollisionUltrasonicId);
    }

    @Override
    public void periodic() {
        mCurrentDistance = getUltrasonicDistance();
    }

    public double getSlowdownScaleFactor() {
        if(!mCollisionEnabled) {
            return 1.0;
        } else if(getCurrentDistance() <= kCollisionStandoffSlowdown) {
            return getCurrentDistance() / kCollisionStandoffSlowdown;
        } else {
            return 1.0;
        }
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

    public static CollisionAvoidanceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CollisionAvoidanceSubsystem();
        }
        return mInstance;
    }
}


package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollisionAvoidanceConstants;

/**
 * Controller for Collision Avoidance Speed Control
 * @author Shreyas Prasad
 */
public class CollisionAvoidanceSubsystem extends SubsystemBase implements SubsystemInterface {

    private static CollisionAvoidanceSubsystem mInstance;

    private boolean mCollisionEnabled;
    private double mCurrentDistance;
    private double mSelectedStandoffSlowdown;

    private final MedianFilter mMedianFilter = new MedianFilter(CollisionAvoidanceConstants.kNumberOfDistanceSamples);
    private final AnalogInput mUltrasonic;

    /**
     * Constructor for CollisionAvoidanceSubsystem Class
     */
    private CollisionAvoidanceSubsystem() {
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

    /**
     * Gets the Ratio the Drivebase Speed should be scaled down by
     * <p>
     * Proportional to the proximity to the object to be collided into
     * @return speed proportion as calculated by the distance from the object; 1.0 if no collision detected
     */
    public double getSlowdownScaleFactor() {
        if(!mCollisionEnabled) {
            return 1.0;
        } else if(getCurrentDistanceInches() <= mSelectedStandoffSlowdown) {
            return getCurrentDistanceInches() / mSelectedStandoffSlowdown;
        } else {
            return 1.0;
        }
    }

    /**
     * Reduce maximum distance before Collision Avoidance is enacted to allow the Robot to get close to the Control Panel when Wheel Controller is deployed
     */
    public void setColorWheelStandoff() {
        mSelectedStandoffSlowdown = CollisionAvoidanceConstants.kColorWheelStandoffSlowdownInches;
    }
    
    /**
     * Set maximum distance before Collision Avoidance activates back to the default
     */
    public void setStandardStandoff() {
        mSelectedStandoffSlowdown = CollisionAvoidanceConstants.kCollisionStandoffSlowdownInches;
    }

    /**
     * Enables Collision Avoidance
     */
    public void enableCollisionAvoidance() {
        mCollisionEnabled = true;
    }

    /**
     * Disables Collision Avoidance
     */
    public void disableCollisionAvoidance() {
        mCollisionEnabled = false;
    }

    /**
     * Gets if Collision Avoidance is Enabled
     * @return <i> true </i> if Collision Avoidance is Enabled; <i> false </i> otherwise
     */
    public boolean isCollisionAvoidanceEnabled() {
        return mCollisionEnabled;
    }

    /**
     * Gets current distance from Robot to any obstacle
     * @return Distance in Inches from Ultrasonic on Robot to obstacle
     */
    public double getCurrentDistanceInches() {
        return mCurrentDistance;
    }

    /**
     * Gets distance from the ultrasonic filtered with a median filter to reduce noise
     * @return Distance in Inches from ultrasonic after filtering
     */
    private double getUltrasonicDistanceInches() {
        return mMedianFilter.calculate(mUltrasonic.getValue() * CollisionAvoidanceConstants.kUltrasonicValueToInches);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(mInstance);
        SmartDashboard.putBoolean("Collision Avoidance Enabled: ", mCollisionEnabled);
    }

    @Override
    public void runTest() {
        BooleanSupplier collisionAvoidanceEnabled = () -> mCollisionEnabled;
        SmartDashboard.putData("Toggle Collision Avoidance", 
        new ConditionalCommand((new InstantCommand(() -> mCollisionEnabled = false)), (new InstantCommand(() -> mCollisionEnabled = true)), collisionAvoidanceEnabled));
    }

    /**
     * @return CollisionAvoidanceSubsystem Singleton Instance
     */
    public synchronized static CollisionAvoidanceSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new CollisionAvoidanceSubsystem();
        }
        return mInstance;
    }
}


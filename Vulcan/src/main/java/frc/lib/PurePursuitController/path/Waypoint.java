package frc.lib.PurePursuitController.path;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Informational Object containing a point, target velocity, and curvature at the point
 * 
 * @author Shreyas Prasad
 */
public class Waypoint {

    private final Translation2d mPosition;
    private final double mTargetVelocity;
    private final double mCurvature;

    /**
     * Waypoint Constructor
     * 
     * @param position Translation2d point
     * @param targetVelocity Target Velocity
     * @param curvature Curvature at point
     */
    public Waypoint(Translation2d position, double targetVelocity, double curvature) {
        mPosition = position;
        mTargetVelocity = targetVelocity;
        mCurvature = curvature;
    }

    /**
     * Waypoint Constructor
     * <p>
     * Defaults Target Velocity and Curvature to 0
     * @param position Translation2d point
     */
    public Waypoint(Translation2d position) {
        mPosition = position;
        mTargetVelocity = 0.0;
        mCurvature = 0.0;
    }

    /**
     * Gets Position
     * 
     * @return Translation2d Point
     */
    public Translation2d getPosition() {
        return mPosition;
    }

    /**
     * Gets Target Velocity
     * 
     * @return Target Velocity
     */
    public double getTargetVelocity() {
        return mTargetVelocity;
    }

    /**
     * Gets Curvature
     * 
     * @return Curvature at Point
     */
    public double getCurvature() {
        return mCurvature;
    }
}
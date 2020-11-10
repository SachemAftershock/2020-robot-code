package frc.lib.PurePursuitController.path;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Class representing a Line, a linear distance between two Waypoints
 * 
 * @author Shreyas Prasad
 */
public class Line {
    
    private final Waypoint mStart;
    private final Waypoint mEnd;
    private final Translation2d mUnitVector;
    private final double mSlope;
    private final double mLength;

    /**
     * Line Constructor
     * 
     * @param start Starting Waypoint
     * 
     * @param end Ending Waypoint
     */
    public Line(Waypoint start, Waypoint end) {
        mStart = start;
        mEnd = end;

        Translation2d startPoint = mStart.getPosition();
        double startX = startPoint.getX();
        double startY = startPoint.getY();

        Translation2d endPoint = mEnd.getPosition();
        double endX = endPoint.getX();
        double endY = endPoint.getY();

        double deltaX = endX - startX;
        double deltaY = endY - startY;

        mSlope = (deltaY) / (deltaX);

        mLength = startPoint.getDistance(endPoint);

        mUnitVector = new Translation2d(deltaX, deltaY).div(mLength);
    }

    /**
     * Line Constructor
     * 
     * @param start Starting Translation2d Point
     * 
     * @param end Ending Translation2d Point
     */
    public Line(Translation2d start, Translation2d end) {
        mStart = new Waypoint(start);
        mEnd = new Waypoint(end);

        Translation2d startPoint = mStart.getPosition();
        double startX = startPoint.getX();
        double startY = startPoint.getY();

        Translation2d endPoint = mEnd.getPosition();
        double endX = endPoint.getX();
        double endY = endPoint.getY();

        double deltaX = endX - startX;
        double deltaY = endY - startY;

        mSlope = (deltaY) / (deltaX);

        mLength = startPoint.getDistance(endPoint);

        mUnitVector = new Translation2d(deltaX, deltaY).div(mLength);
    }

    /**
     * Gets Starting Waypoint
     * 
     * @return Starting Waypoint
     */
    public Waypoint getStart() {
        return mStart;
    }

    /**
     * Gets Ending Waypoint
     * 
     * @return Ending Waypoint
     */
    public Waypoint getEnd() {
        return mEnd;
    }

    /**
     * Gets Slope of the Line
     * 
     * @return Slope of the Line
     */
    public double getSlope() {
        return mSlope;
    }

    /**
     * Gets Length of the Line
     * 
     * @return Length of the Line
     */
    public double getLength() {
        return mLength;
    }

    /**
     * Gets Unit Vector
     * 
     * @return Translation2d Vector with Length 1 in the direction of the Line
     */
    public Translation2d getUnitVector() {
        return mUnitVector;
    }
}
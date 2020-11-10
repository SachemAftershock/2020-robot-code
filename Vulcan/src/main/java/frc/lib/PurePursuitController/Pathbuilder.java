package frc.lib.PurePursuitController;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.lib.PurePursuitController.path.Line;
import frc.lib.PurePursuitController.path.Path;
import frc.lib.PurePursuitController.path.Trajectory;

/**
 * Class of Static Utility Functions to Build Paths and Trajectories
 * 
 * @author Shreyas Prasad
 */
public class Pathbuilder {
    
    private static final double kPointSpacing = 6.0;

    private static final double kDefaultMaxVelocity = 10; //TODO: Find value; might want to tune per path
    private static final double kDefaultMaxAccel = 2; //TODO: Find value; might want to tune per path
    private static final double kCurvatureSlowConstant = 2; //TODO: Find value; paper says around 1-5

    /**
     * Builds a Path from a list of Translation2d points that lie on the path
     * 
     * @param points List of Translation2d Points that the path must cross
     * 
     * @return Not Smoothed Path that crosses every point in points
     */
    public static Path buildPath(List<Translation2d> points) {
        List<Line> lines = new ArrayList<Line>();
        for(int i=0;i<points.size()-1;i++) {
            lines.add(new Line(points.get(i), points.get(i+1)));
        }

        List<Translation2d> newPoints = new ArrayList<Translation2d>();
        for(final Line line : lines) {
            final double numPoints = Math.ceil(line.getLength() / kPointSpacing);
            final Translation2d vector = line.getUnitVector().times(kPointSpacing);
            for(int i=0;i<numPoints;i++) {
                Translation2d positonVector = line.getStart().getPosition().plus(vector.times(i));
                newPoints.add(positonVector);
            }
            newPoints.add(line.getEnd().getPosition());
        }
        return new Path(newPoints);
    }

    /**
     * Generates a Trajectory from a path, maximum possible velocity, and maximum possible acceleration
     * 
     * @param path Built Path
     * 
     * @param maxVelocity Maximum Possible Velocity the robot should take on the Path
     * 
     * @param maxAccel Maximum Possible Acceleration the robot should take on the Path
     * 
     * @return Trajectory
     */
    public static Trajectory generateTrajectory(Path path, final double maxVelocity, final double maxAccel) {
        List<Double> curvatures = calculateCurvatureList(path.getVectorList());

        List<Double> maxVelocityList = calculateMaxVelocityList(curvatures, maxVelocity);

        List<Double> accelLimitedVelocityList = calculateAccelerationLimitedVelocityList(path, maxVelocityList, maxAccel);
        
        return new Trajectory(path, accelLimitedVelocityList, curvatures);
    }

    /**
     * Generates a Trajectory from a path, maximum possible velocity, and maximum possible acceleration
     * 
     * <p>
     * <b> Uses default maximum Velocity and maximum Acceleration </b>
     * </p>
     * 
     * @param path Built Path
     * 
     * @return Trajectory
     */
    public static Trajectory generateTrajectory(Path path) {
        return generateTrajectory(path, kDefaultMaxVelocity, kDefaultMaxAccel);
    }

    /**
     * Calculates curvature (Inverse of radius) at each vector point
     * 
     * @param vectors List of Translation2d Vector points
     * 
     * @return List of Curvatures at each Vector Point
     */
    private static List<Double> calculateCurvatureList(List<Translation2d> vectors) {
        List<Double> curvatureList = new ArrayList<Double>();
        curvatureList.add(0.0);//Start has 0.0 curvature

        for(int i=1;i<vectors.size()-1;i++) {
            Translation2d point1 = vectors.get(i-1);
            double x1 = point1.getX();
            double y1 = point1.getY();

            Translation2d point2 = vectors.get(i);
            double x2 = point2.getX();
            double y2 = point2.getY();

            Translation2d point3 = vectors.get(i+1);
            double x3 = point3.getX();
            double y3 = point3.getY();

            //Prevent division by zero
            if(x1 == x2) {
                x1 += 1E-3;
            }

            double k1 = 0.5 * (Math.pow(x1,2) + Math.pow(y1,2) - Math.pow(x2,2) - Math.pow(y2,2)) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);

            double b = 0.5 * (Math.pow(x2,2) - 2 * x2 * k1 * Math.pow(y2,2) - Math.pow(x3,2) + 2 * x3 * k1 - Math.pow(y3,2)) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;
            
            double radius = Math.hypot(x1 - a, y1 - b);
            double curvature = 1 / radius;

            curvatureList.add(curvature);
        }

        curvatureList.add(0.0); //End has 0.0 curvature

        return curvatureList;
    }

    /**
     * Calculates a list of maximum velocities for each point based on the inverse of the curvature
     * 
     * @param curvatures list of curvatures at each point
     * 
     * @param maxVelocity upper bound for velocity
     * 
     * @return list of maximum velocities at each point based on the inverse of the curvature
     */
    private static List<Double> calculateMaxVelocityList(List<Double> curvatures, final double maxVelocity) {
        List<Double> maxVelocityList = new ArrayList<Double>();

        maxVelocityList.add(maxVelocity);//start and end have 0.0 curvature; avoid division by zero
        for(int i=1;i<curvatures.size()-1;i++) {
            maxVelocityList.add(Math.min(maxVelocity, kCurvatureSlowConstant / curvatures.get(i)));
        }
        maxVelocityList.add(maxVelocity);

        return maxVelocityList;
    }

    /**
     * @param path
     * @param maxVelocityList
     * @param maxAccel
     * @return
     */
    private static List<Double> calculateAccelerationLimitedVelocityList(Path path, List<Double> maxVelocityList, final double maxAccel) {
        final int lastIndex = maxVelocityList.size()-1;

        List<Double> accelLimitedVelocityList = new ArrayList<Double>(maxVelocityList);

        accelLimitedVelocityList.set(lastIndex, 0.0);

        for(int i=lastIndex-1;i>=0;i--) {
            double distance = path.getDistanceFromEnd(i);
            double previousVelocity = accelLimitedVelocityList.get(i+1);

            double calculatedVelocity = Math.sqrt(Math.pow(previousVelocity, 2) + 2 * maxAccel * distance);

            //check to see if the accel actually lowered the velocity, if not, keep as max velocity
            double newVelocity = Math.min(calculatedVelocity, maxVelocityList.get(i));

            accelLimitedVelocityList.set(i, newVelocity);
        }

        return accelLimitedVelocityList;
    }
}
package frc.lib.PurePursuitController.path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * List of Vectors representing the Path for the Robot
 * 
 * @author Shreyas Prasad
 */
public class Path {
    
    private List<Translation2d> mAllVectors;

    //May want to tune weight smooth per path; requires testing
    //Paper found between 0.75 and 0.98 are good values
    private static final double kDefaultWeightSmooth = 0.8;
    private static final double kDefaultWeightData = 1 - kDefaultWeightSmooth;
    private static final double kDefaultTolerance = 0.001;

    /**
     * Path Constructor
     * 
     * @param vectors List of Translation2d Vectors
     */
    public Path(List<Translation2d> vectors) {
        mAllVectors = vectors;
    }

    /**
     * Clones Path object
     * 
     * @param otherPath Path to clone
     */
    public Path(Path otherPath) {
        mAllVectors = otherPath.getVectorList();
    }

    /**
     * Gets Vector List
     * 
     * @return List of Translatino2d Vectors
     */
    public List<Translation2d> getVectorList() {
        return mAllVectors;
    }

    /**
     * Starts from Last Point, and calculates distance backwards up to <i> index </i>
     * 
     * @param index Index to calculate distance up to from the last point
     * @return distance from final point to <i> index </i>
     */
    public double getDistanceFromEnd(int index) {
        final int lastIndex = mAllVectors.size()-1;
        double distance  = mAllVectors.get(lastIndex).getNorm();

        if(index == lastIndex) {
            return distance;
        }

        for(int i=lastIndex-1;i >= index;i--) {
            distance += mAllVectors.get(i+1).getDistance(mAllVectors.get(i));
        }
        return distance;
    }

    /**
     * Algorithm adapted from FRC Team 2168
     * <p>
	 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
	 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	 * converge. If this happens, try increasing the tolerance level.
	 * </p>
	 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met. 
     * 
	 * @param weightData
	 * @param weightSmooth Ratio of how much to smooth the path [0,1]
	 * @param tolerance
	 */
	public void smoothPath(double weightData, double weightSmooth, double tolerance) {
        List<Translation2d> smoothedList = new ArrayList<Translation2d>(mAllVectors);

		double change = tolerance;
		while(change >= tolerance) {
            change = 0.0;
            
			for(int i=1; i<mAllVectors.size()-1; i++) {

				double auxX = smoothedList.get(i).getX();
				double x =  smoothedList.get(i).getX() + weightData * (mAllVectors.get(i).getX() - smoothedList.get(i).getX()) + weightSmooth * (smoothedList.get(i-1).getX() + smoothedList.get(i+1).getX() - (2.0 * smoothedList.get(i).getX()));
                change += Math.abs(auxX - x);

                double auxY = smoothedList.get(i).getY();
				double y =  smoothedList.get(i).getY() + weightData * (mAllVectors.get(i).getY() - smoothedList.get(i).getY()) + weightSmooth * (smoothedList.get(i-1).getY() + smoothedList.get(i+1).getY() - (2.0 * smoothedList.get(i).getY()));
                change += Math.abs(auxY - y);	
                
                smoothedList.set(i, new Translation2d(x, y));
            }				
		}

        mAllVectors = smoothedList;
    }

    /**
     * Algorithm adapted from FRC Team 2168
     * <p>
	 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
	 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	 * converge. If this happens, try increasing the tolerance level.
	 * </p>
	 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met. 
     * 
	 * <p>
     * <b> Uses default Weight Data, Weight Smooth, and Tolerance </b>
     * </p>
	 */
	public void smoothPath() {
        this.smoothPath(kDefaultWeightData, kDefaultWeightSmooth, kDefaultTolerance);
    }
}
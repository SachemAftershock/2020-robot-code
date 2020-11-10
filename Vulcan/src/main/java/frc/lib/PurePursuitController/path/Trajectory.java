package frc.lib.PurePursuitController.path;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Path with generated velocities
 * 
 * @author Shreyas Prasad
 */
public class Trajectory {

    private List<Waypoint> mAllWaypoints;

    /**
     * Trajectory Constructor
     * @param waypoints List of Waypoints
     */
    public Trajectory(List<Waypoint> waypoints) {
        mAllWaypoints = waypoints;
    }

    /**
     * Trajectory Constructor
     * 
     * @param path Path for Trajectory to follow
     * @param velocityList List of Generated Velocities for each point in the Path
     * @param curvatureList List of Curvatures for each point in the Path
     */
    public Trajectory(Path path, List<Double> velocityList, List<Double> curvatureList) {
        List<Translation2d> vectorList = path.getVectorList();
        for(int i=0;i<vectorList.size();i++) {
            mAllWaypoints.add(new Waypoint(vectorList.get(i), velocityList.get(i), curvatureList.get(i)));
        }
    }

    /**
     * Gets List of Waypoints
     * 
     * @return List of Waypoints
     */
    public List<Waypoint> getWaypoints() {
        return mAllWaypoints;
    }
}
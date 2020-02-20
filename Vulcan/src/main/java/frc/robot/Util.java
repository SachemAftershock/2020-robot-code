package frc.robot;

/**
 * Class to hold utility methods to be used throughout the codebase
 */
public class Util {

    /**
     * Applies a deadband to the value with given tolerance
     * 
     * @param value the value to apply the deadband to
     * 
     * @param tolerance minimum value
     * 
     * @return <b> value </b> if greater than <b> tolerance </b>; 0.0 otherwise
     */
    public static double deadband(double value, double tolerance) {
        return Math.abs(value) >= tolerance ? value : 0.0;
    }
    
    /**
	* Gets rotational error on [-180, 180]
	* 
    * @param alpha First angle
    
    * @param beta Second Angle
    
	* @return Rotational error
	*/
    public static double rotationalError(double alpha, double beta) {
        double ret = alpha - beta;
        if (ret < -180) {
            ret += 360;
        }
        if (ret > 180) {
            ret -= 360;
        }
        return -ret;
    }

    /**
     * Converts angles from [0,360] to [-180, 180]
     * 
     * @param theta Angle in range [0,360]
     * 
     * @return Angle converted to [-180,180]
     */
    public static double normalizeAngle(double theta) {
        if(theta > 180) {
            theta -= 360;
        } else if(theta < -180) {
            theta += 360;
        }

        return theta;
    }
}
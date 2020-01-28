package frc.robot;

public class Util {

    public static double deadband(double value, double tolerance) {
        return Math.abs(value) >= tolerance ? value : tolerance;
    }
    
    /**
	* Gets rotational error on [-180, 180]
	* 
	* @param alpha
	*            First angle
	* @param beta
	*            Second Angle
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

    public static double normalizeAngle(double theta) {
        if(theta > 180) {
            theta -= 360;
        } else if(theta < -180) {
            theta += 360;
        }

        return theta;
    }

    public static double convertRPMToRadiansPerSecond(double speedInRPM) {
        return speedInRPM / 60 * 2 * Math.PI;
    }
}
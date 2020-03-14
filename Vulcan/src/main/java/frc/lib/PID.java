package frc.lib;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class for creating PID Controllers
 * 
 * @author Rohan Bapat
 * @author Shreyas Prasad
*/
public class PID {

    private boolean mPaused;
    private double mIntegral, mDerivative, mError, mPreviousError;
    private double[] mGains;
    private Timer mTimer;

    /**
     * Instantiate PID Object
     */
    public PID() {
        mGains = new double[3];
        mIntegral = 0.0;
        mDerivative = 0.0;
        mError = 0.0;
        mPreviousError = 0.0;

        mPaused = false;

        mTimer = new Timer();
    }

    /**
     * Calculates PID Output for Linear models
     * @param current the current linear value
     * @param setpoint the desired linear value
     * @return the output of the PID [-1,1]
     */
    public double update(double current, double setpoint) {
        mError = setpoint - current;
        double output = updateError(mError);
        return output;
    }

    /**
     * Calculates PID of Rotational Error given angular measures in [-180, 180]
     * @param current the current angular value
     * @param setpoint the desired angular value
     * @return PID output [-1,1]
     */
    public double updateRotation(double current, double setpoint) {
        mError = Util.rotationalError(current, setpoint);
        return updateError(mError);
    }

    /**
     * Gets output of PID Calculation
     * @param error difference between setpoint & current values
     * @return the output of the PID in [-1,1]
     */
    private double updateError(double error) {
        mError = error;
        mIntegral += error;
        mDerivative = error - mPreviousError;

        double output = mGains[0] * error + mGains[1] * mIntegral + mGains[2] * mDerivative;
        mPreviousError = error;
        if(Math.abs(output) > 1.0) {
            output /= output < 0 ? -output : output;
        }

        return output;
    }

    /**
     * Begin PID. Start Runtime Timer & Initalize Gains
     * @param gains Array of P,I,D gain values
     */
    public void start(double[] gains) {
        mGains = gains;
        mIntegral = 0.0;
        mDerivative = 0.0;
        mError = 0.0;
        mPreviousError = 0.0;
        mTimer.start();
    }

    /**
     * Resets Integral and Derivative Terms when PID is not running to prevent Integral Windup
     */
    public void pausePID() {
        mIntegral = 0.0;
        mDerivative = 0.0;
        mPaused = true;
    }

    /**
     * Resumes PID
     */
    public void resumePID() {
        mTimer.reset();
        mPaused = false;
    }

    /**
     * Gets if PID Controller is currently paused
     * @return <i>true</i> if PID is currently paused; <i>false</i> otherwise
     */
    public boolean isPaused() {
        return mPaused;
    }

    /**
     * Gets Error Value
     * @return difference between setpoint & current value
     */
    public double getError() {
        return mError;
    }

    /**
     * Gets the runtime of the PID Controller
     * @return seconds since PID has started or been resumed
     */
    public double getRunTime() {
        return mTimer.get();
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class PID {

    private final double kEpsilon;
    private boolean mPaused;
    private double mIntegral, mDerivative, mError, mPreviousError;
    private double[] mGains;
    private Timer mTimer;

    public PID(double epsilon) {
        mGains = new double[3];
        mIntegral = 0.0;
        mDerivative = 0.0;
        mError = 0.0;
        mPreviousError = 0.0;

        mPaused = false;

        kEpsilon = epsilon;

        mTimer = new Timer();
    }

    public double update(double current, double setpoint) {
        mError = setpoint - current;
        double output = updateError(mError);
        return output;
    }

    public double updateRotation(double current, double setpoint) {
        mError = Util.rotationalError(current, setpoint);
        return updateError(mError);
    }

    private double updateError(double error) {
        if(Math.abs(error) <= kEpsilon) {
            return 0.0;
        }
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

    public void start(double[] gains) {
        mGains = gains;
        mIntegral = 0.0;
        mDerivative = 0.0;
        mError = 0.0;
        mPreviousError = 0.0;
        mTimer.start();
    }

    public void pausePID() {
        mIntegral = 0.0;
        mDerivative = 0.0;
        mPaused = true;
    }

    public void resumePID() {
        mTimer.reset();
        mPaused = false;
    }

    public boolean isPaused() {
        return mPaused;
    }


    public double getError() {
        return mError;
    }

    public double getRunTime() {
        return mTimer.get();
    }
}

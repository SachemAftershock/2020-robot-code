package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerRumble;
import frc.robot.Lidar;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperstructureConstants.ShooterConstants;

/**
 * Shooter Subsystem
 * 
 * @author Shreyas Prasad
 * 
 * @see SuperstructureSubsystem
 * 
 * @see TurretSubsystem
 */
public class ShooterSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ShooterSubsystem mInstance;

    private final Limelight mLimelight;
    
    private final CANSparkMax mShooter;
    private final Lidar mLowLidar;//, mHighLidar;

    private final CANEncoder mEncoder;
    private final CANPIDController mPid;

    private double mTargetRPM, mCurrentPredictedDistance;

    private int mTargetFalloutDelayCounter;

    /**
     * Constructor for ShooterSubsystem Class
     */
    private ShooterSubsystem() {
        mShooter = new CANSparkMax(ShooterConstants.kLauncherMotorId, MotorType.kBrushless);
        mShooter.restoreFactoryDefaults();
        mShooter.setMotorType(MotorType.kBrushless);
        mShooter.setIdleMode(IdleMode.kCoast);
        mShooter.setInverted(true);
        mShooter.burnFlash();

        mLowLidar = new Lidar(new DigitalInput(ShooterConstants.kLowLidarId));
        //mHighLidar = new Lidar(new DigitalInput(ShooterConstants.kHighLidarId));

        mEncoder = mShooter.getEncoder();

        mPid = mShooter.getPIDController();

        mPid.setP(ShooterConstants.kGains[0], ShooterConstants.kPidId);
        mPid.setI(ShooterConstants.kGains[1], ShooterConstants.kPidId);
        mPid.setD(ShooterConstants.kGains[2], ShooterConstants.kPidId);
        mPid.setFF(ShooterConstants.kGains[3], ShooterConstants.kPidId);
        mPid.setIZone(ShooterConstants.kGains[4], ShooterConstants.kPidId);
        mPid.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
        mPid.setSmartMotionMaxVelocity(ShooterConstants.kMaxVelocity, ShooterConstants.kPidId);
        mPid.setSmartMotionMaxAccel(ShooterConstants.kLowAccelerationRPMPerSecond, ShooterConstants.kPidId);
        mPid.setSmartMotionAllowedClosedLoopError(ShooterConstants.kShooterSpeedEpsilon, ShooterConstants.kPidId);

        mLimelight = LimelightManagerSubsystem.getInstance().getShooterLimelight();
    }

    @Override
    public void init() {
        mShooter.set(0.0);
        mPid.setSmartMotionMaxAccel(ShooterConstants.kLowAccelerationRPMPerSecond, ShooterConstants.kPidId);
    }

    @Override
    public void periodic() {
    }
    
    /**
     * Runs Velocity PID on Neo Shooter Motor to reach the calculated RPM
     * <p>
     * Gets RPM by Polynomial Regression: {@link frc.robot.Constants.SuperstructureConstants.ShooterConstants#kShooterPolynomial}
     * <p>
     * If Target falls out of view for 2.5s, goes to lowest RPM in regression table
     */
    public void reachCalculatedTargetRPM() {
        if(mLimelight.isTarget() || isLowLidarCorrect()) {
            mTargetRPM = getTargetRPM();
            mTargetFalloutDelayCounter = 0;
        } else if(mTargetFalloutDelayCounter > 2500 / 20) {
            mTargetRPM = ShooterConstants.kDistanceInToRpmLUT[ShooterConstants.kMinRPMCellIndex][ShooterConstants.kRPMIndex];
            mTargetFalloutDelayCounter = 0;
        } else {
            mTargetFalloutDelayCounter++;
        }

        if(mTargetRPM < 1.0) {
            (new ControllerRumble(RobotContainer.getInstance().getControllerPrimary(), 1)).start();
            return;
        }
        
        PowerSubsystem.getInstance().stopCompressor();

        //TODO: Find out if I need this
        /*
        if(mEncoder.getVelocity() > ShooterConstants.kVelocityAccelShiftThresholdRPM && mPid.getSmartMotionMaxAccel(ShooterConstants.kPidId) == ShooterConstants.kLowAccelerationRPMPerSecond) {
            mPid.setSmartMotionMaxAccel(ShooterConstants.kHighAccelerationRPMPerSecond, ShooterConstants.kPidId);
        } else if (mEncoder.getVelocity() > ShooterConstants.kVelocityAccelShiftThresholdRPM && mPid.getSmartMotionMaxAccel(ShooterConstants.kPidId) == ShooterConstants.kHighAccelerationRPMPerSecond) {
            mPid.setSmartMotionMaxAccel(ShooterConstants.kLowAccelerationRPMPerSecond, ShooterConstants.kPidId);
        }
        */

        mPid.setReference(mTargetRPM, ControlType.kSmartVelocity, ShooterConstants.kPidId);
    }

    public void setTargetRPM(double setpoint) {
        mTargetRPM = setpoint;
        mPid.setReference(mTargetRPM, ControlType.kSmartVelocity, ShooterConstants.kPidId);
    } 

    /**
     * Stops Shooter Motor
     */
    public void stopShooter() {
        PowerSubsystem.getInstance().startCompressor();
        mShooter.set(0.0);
    }

    /**
     * Checks if Shooter is at the Target RPM
     * 
     * @return <i> true </i> if at Target RPM; <i> false </i> otherwise
     */
    public synchronized boolean isAtTargetRPM() {
        double v = mEncoder.getVelocity();
        return Math.abs(v - mTargetRPM) <= ShooterConstants.kShooterSpeedEpsilon || v > mTargetRPM; //TODO: Find out if I need this second part
        
    }

    private double getTargetRPM() {
        if(mLimelight.isTarget()) {
            mCurrentPredictedDistance = ShooterConstants.kTyDistanceRegression.predict(mLimelight.getTy());
        } else if(isLowLidarCorrect()) {
            mCurrentPredictedDistance = mLowLidar.getDistanceIn();
        } else {
            return 0.0;
        }

        if(mCurrentPredictedDistance <= ShooterConstants.kDistanceInToRpmLUT[0][0]) { 
            return 0.0; //Current Range is below min range
        }

        for(int i = 1; i < ShooterConstants.kDistanceInToRpmLUT.length; i++) {
            if(mCurrentPredictedDistance < ShooterConstants.kDistanceInToRpmLUT[i][0]) {
                double y1 = ShooterConstants.kDistanceInToRpmLUT[i][ShooterConstants.kRPMIndex];
                double y0 = ShooterConstants.kDistanceInToRpmLUT[i-1][ShooterConstants.kRPMIndex];

                double x1 = ShooterConstants.kDistanceInToRpmLUT[i][ShooterConstants.kDistanceIndex];
                double x0 = ShooterConstants.kDistanceInToRpmLUT[i-1][ShooterConstants.kDistanceIndex];

                mTargetRPM = (y1 - y0) / (x1 - x0) * (mCurrentPredictedDistance - x0) + y0;
                return mTargetRPM;
            }
        }
        
        return 0.0;
    }

    private boolean isLowLidarCorrect() {
        return mLowLidar.getDistanceIn() > ShooterConstants.kLowLidarMin && mLowLidar.getDistanceIn() < ShooterConstants.kLowLidarMax;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Shooter Velocity", mEncoder.getVelocity());
        SmartDashboard.putNumber("Num Shooter Velocity", mEncoder.getVelocity());
        SmartDashboard.putNumber("Target Velocity", mTargetRPM);
        SmartDashboard.putBoolean("Is at Target RPM", isAtTargetRPM());
        SmartDashboard.putNumber("Low Lidar", mLowLidar.getDistanceIn());
        SmartDashboard.putNumber("Ty Predicted Distance", mCurrentPredictedDistance);
        SmartDashboard.putNumber("Ty", mLimelight.getTy());
    }   

    @Override
    public boolean checkSystem() {
        return true;
    }
    
    /**
     * @return ShooterSubsystem Singleton Instance
     */
    public synchronized static ShooterSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ShooterSubsystem();
        }
        return mInstance;
    }
}


package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    
    private final CANSparkMax mShooter;
    //private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;

    private double mTargetRPM;

    //private int mRumbleDelayCounter, mTargetFalloutDelayCounter;

    public double kP, kI, kD, kIz, kFF;

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

        //mLidar = new Lidar(new DigitalInput(ShooterConstants.kLidarId));

        mShooterEncoder = mShooter.getEncoder();

        mShooterPid = mShooter.getPIDController();

        //mRumbleDelayCounter = 0;
        //mTargetFalloutDelayCounter = 0;

        kP = 0.002;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.0002;

        mShooterPid.setP(kP /*ShooterConstants.kGains[0]*/, ShooterConstants.kPidId);
        mShooterPid.setI(kI /*ShooterConstants.kGains[1]*/, ShooterConstants.kPidId);
        mShooterPid.setD(kD /*ShooterConstants.kGains[2]*/, ShooterConstants.kPidId);
        mShooterPid.setFF(kFF /*ShooterConstants.kGains[3]*/, ShooterConstants.kPidId);
        mShooterPid.setIZone(kIz /*ShooterConstants.kGains[4]*/, ShooterConstants.kPidId);
        mShooterPid.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
        mShooterPid.setSmartMotionMaxVelocity(ShooterConstants.kMaxVelocity, ShooterConstants.kPidId);
        mShooterPid.setSmartMotionMaxAccel(ShooterConstants.kLowAccelerationRPMPerSecond, ShooterConstants.kPidId);
        mShooterPid.setSmartMotionAllowedClosedLoopError(ShooterConstants.kShooterSpeedEpsilon, ShooterConstants.kPidId);
    }

    @Override
    public void init() {
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
    }

    @Override
    public void periodic() {
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { mShooterPid.setP(p); kP = p;}
        if((i != kI)) { mShooterPid.setI(i); kI = i;}
        if((d != kD)) { mShooterPid.setD(d); kD = d;}
        if((iz != kIz)) { mShooterPid.setIZone(iz); kIz = iz;}
        if((ff != kFF)) { mShooterPid.setFF(ff); kFF = ff;}
    }
    
    /**
     * Runs Velocity PID on Neo Shooter Motor to reach the calculated RPM
     * <p>
     * Gets RPM by Polynomial Regression: {@link frc.robot.Constants.SuperstructureConstants.ShooterConstants#kShooterPolynomial}
     * <p>
     * If Target falls out of view for 2.5s, goes to median RPM in regression table
     */
    public void reachCalculatedTargetRPM() {
        /*
        if(LimelightManagerSubsystem.getInstance().getShooterLimelight().isTarget()) {
            mTargetRPM = ShooterConstants.kShooterPolynomial.predict(mLidar.getDistanceIn() / 12.0);
            mTargetFalloutDelayCounter = 0;
        } else if(mTargetFalloutDelayCounter > 2500 / 20) {
            mTargetRPM = ShooterConstants.kDistanceFeetToRpmLUT[(int) (ShooterConstants.kDistanceFeetToRpmLUT.length / 2)][1];
            mTargetFalloutDelayCounter = 0;
        } else {
            mTargetFalloutDelayCounter++;
        }
        */
        PowerSubsystem.getInstance().stopCompressor();
        mTargetRPM = 4250;

        if(mShooterEncoder.getVelocity() > ShooterConstants.kVelocityAccelShiftThresholdRPM) {
            mShooterPid.setSmartMotionMaxAccel(ShooterConstants.kHighAccelerationRPMPerSecond, ShooterConstants.kPidId);
        } else {
            mShooterPid.setSmartMotionMaxAccel(ShooterConstants.kLowAccelerationRPMPerSecond, ShooterConstants.kPidId);
        }
        mShooterPid.setReference(mTargetRPM, ControlType.kSmartVelocity, ShooterConstants.kPidId);
    }

    /**
     * Stops Shooter Motor
     */
    public void stopShooter() {
        PowerSubsystem.getInstance().startCompressor();
        mShooter.set(0.0);
    }

    /**
     * Checks if Shooter is at the Target RPM & Rumbles Controller if at RPM
     * 
     * @return <i> true </i> if at Target RPM; <i> false </i> otherwise
     */
    public synchronized boolean isAtTargetRPM() {
        final boolean isAtTargetRPM = Math.abs(mShooterEncoder.getVelocity() - mTargetRPM) < ShooterConstants.kShooterSpeedEpsilon;
        /*
        if(isAtTargetRPM) {
            if(mRumbleDelayCounter > 1000 / 20) {
               // (new ControllerRumble(RobotContainer.getInstance().getControllerSecondary(), 1)).start();
                mRumbleDelayCounter = 0;
            }
            mRumbleDelayCounter++;
        }
        */
        return isAtTargetRPM;
        
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Shooter Velocity", mShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Target Velocity", mTargetRPM);
        SmartDashboard.putBoolean("Is at Target RPM", isAtTargetRPM());
        //SmartDashboard.putNumber("Lidar Distance Ft", mLidar.getDistanceIn() / 12.0);
    }   

    @Override
    public void runTest() {
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


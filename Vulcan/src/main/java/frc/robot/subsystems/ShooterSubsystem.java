package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControllerRumble;
import frc.robot.Lidar;
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
    
    private final CANSparkMax mShooter;
    //private final WPI_TalonSRX mFeeder;
    private final DoubleSolenoid mLoader;
    private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;

    private double mTargetRPM;

    private int mRumbleDelayCounter, mTargetFalloutDelayCounter;

    /**
     * Constructor for ShooterSubsystem Class
     */
    private ShooterSubsystem() {
        mShooter = new CANSparkMax(ShooterConstants.kLauncherMotorId, MotorType.kBrushless);
        mShooter.restoreFactoryDefaults();
        mShooter.setMotorType(MotorType.kBrushless);
        mShooter.setIdleMode(IdleMode.kCoast); //Brake mode might be really bad
        mShooter.setInverted(false);
        mShooter.burnFlash();

        //mFeeder = new WPI_TalonSRX(ShooterConstants.kFeederMotorId);
        mLoader = new DoubleSolenoid(Constants.kPcmId, ShooterConstants.kLoaderForwardId, ShooterConstants.kLoaderReverseId);

        mLidar = new Lidar(new DigitalInput(ShooterConstants.kLidarId));

        mShooterEncoder = mShooter.getEncoder();

        mShooterPid = new CANPIDController(mShooter);
        mShooterPid.setP(ShooterConstants.kGains[0], ShooterConstants.kPidId);
        mShooterPid.setI(ShooterConstants.kGains[1], ShooterConstants.kPidId);
        mShooterPid.setD(ShooterConstants.kGains[2], ShooterConstants.kPidId);
        mShooterPid.setIZone(ShooterConstants.kGains[3], ShooterConstants.kPidId);
        mShooterPid.setFF(ShooterConstants.kGains[4], ShooterConstants.kPidId);
        mShooterPid.setOutputRange(-1.0, 1.0);

        mRumbleDelayCounter = 0;
        mTargetFalloutDelayCounter = 0;
    }

    @Override
    public void init() {
    }
    
    /**
     * Runs Velocity PID on Neo Shooter Motor to reach the calculated RPM
     * <p>
     * Gets RPM by Polynomial Regression: {@link frc.robot.Constants.SuperstructureConstants.ShooterConstants#kShooterPolynomial}
     * <p>
     * If Target falls out of view for 2.5s, goes to median RPM in regression table
     */
    public void reachCalculatedTargetRPM() {
        if(LimelightManagerSubsystem.getInstance().getShooterLimelight().isTarget()) {
            mTargetRPM = ShooterConstants.kShooterPolynomial.predict(mLidar.getDistanceIn() / 12.0);
            mTargetFalloutDelayCounter = 0;
        } else if(mTargetFalloutDelayCounter > 2500 / 20) {
            mTargetRPM = ShooterConstants.kDistanceFeetToRpmLUT[(int) (ShooterConstants.kDistanceFeetToRpmLUT.length / 2)][1];
            mTargetFalloutDelayCounter = 0;
        } else {
            mTargetFalloutDelayCounter++;
        }
        mShooterPid.setReference(mTargetRPM, ControlType.kVelocity, ShooterConstants.kPidId);
    }

    /**
     * Stops Shooter Motor
     */
    public void stopShooter() {
        mShooter.set(0.0);
    }

    /**
     * Checks if Shooter is at the Target RPM & Rumbles Controller if at RPM
     * 
     * @return <i> true </i> if at Target RPM; <i> false </i> otherwise
     */
    public synchronized boolean isAtTargetRPM() {
        final boolean isAtTargetRPM = Math.abs(mShooterEncoder.getVelocity() - mTargetRPM) < ShooterConstants.kShooterSpeedEpsilon;
        if(isAtTargetRPM) {
            if(mRumbleDelayCounter > 1000 / 20) {
                (new ControllerRumble(RobotContainer.getInstance().getControllerSecondary(), 1)).start();
                mRumbleDelayCounter = 0;
            }
            mRumbleDelayCounter++;
        }
        return isAtTargetRPM;
    }

    /**
     * Loader Piston pushes Power Cell in Chamber to barrel
     */
    public void loadBall() {
        mLoader.set(Value.kForward);
    }
    
    /**
     * Loader Piston opens to allow another ball to be pushed into the barrel
     */
    public void openBallLoader() {
        mLoader.set(Value.kReverse);
    }

    //Reimplement when the Piston Loader is replaced by the feeder wheel
    /*
    public void startFeeder() {
        mFeeder.set(ControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        mFeeder.set(ControlMode.PercentOutput, 0.0);
    }
    */

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Shooter Velocity", mShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Target Velocity", mTargetRPM);
        SmartDashboard.putNumber("Lidar Distance Ft", mLidar.getDistanceIn() / 12.0);
    }

    @Override
    public void runTest() {
        // TODO Auto-generated method stub
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


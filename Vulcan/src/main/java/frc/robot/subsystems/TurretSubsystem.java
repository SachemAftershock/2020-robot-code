package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PID;
import frc.robot.Constants.SuperstructureConstants.TurretConstants;

/**
 * Turret Subsystem
 * 
 * @author Shreyas Prasad
 * 
 * @see SuperstructureSubsystem
 * 
 * @see ShooterSubsystem
 * 
 */
public class TurretSubsystem extends SubsystemBase implements SubsystemInterface {

    private static TurretSubsystem mInstance;

    private WPI_TalonSRX mTurret;
    private DutyCycleEncoder mEncoder;
    private PID mPid;

    private boolean mAutoTargetingEnabled;
    private ShootingTarget mSelectedTarget;

    /**
     * Constructor for TurretSubsystem Class
     */
    private TurretSubsystem() {
        mTurret = new WPI_TalonSRX(TurretConstants.kTurretMotorId);
        mTurret.setNeutralMode(NeutralMode.Brake);

        mEncoder = new DutyCycleEncoder(new DigitalInput(TurretConstants.kTurretEncoderDioId));
        mEncoder.setDistancePerRotation(TurretConstants.kTurretDegreesPerEncoderRotation);
        mEncoder.reset();

        mPid = new PID();
        mPid.start(TurretConstants.kGains);

        mAutoTargetingEnabled = true;

        mSelectedTarget = ShootingTarget.eHighTarget;
    }

    @Override
    public void init() {
    }

    /**
     * Turret Auto Targeting Processor
     * <p>
     * Uses Limelight tx to aim onto target: {@link LimelightManagerSubsystem}
     */
    @Override
    public void periodic() {
        if(mAutoTargetingEnabled) {
            final double tx = LimelightManagerSubsystem.getInstance().getShooterLimelight().getTx();
            final double robotAziumth = DriveSubsystem.getInstance().getHeading();  // TODO: make sure zero means heading is downfield.
            final double turretAzimuth = mEncoder.getDistance();  // Turret Azimuth in -180..180, zero is turrent inline robot forward.
            double turretSetpointInDegrees = 0.0; 
            // -180..180deg where 0deg is turret inline looking forward on robot.
            // Simply change this setpoint at any time to redirect the turret.

            final boolean isLimelightAimedDownfield = ((robotAziumth + turretAzimuth) > -90.0) && ((robotAziumth + turretAzimuth) < 90.0);

            if ((tx >= TurretConstants.kMaxTx) || !isLimelightAimedDownfield) {
                // No tape in field of view of limelight, or we're seeing the wrong target,
                // so make decisions based on robot orientation to field.
                turretSetpointInDegrees = -robotAziumth;
            } else {
                // compute a value that may be out of -180..180 range
                double interimTurretAzimuth = turretAzimuth + tx;
                // Ensure shift it back to be within range, which also guards against 
                // over rotating (will swing around thru zero). 
                if (interimTurretAzimuth > 180.0) {
                    turretSetpointInDegrees = interimTurretAzimuth - 360.0;  
                } else if (interimTurretAzimuth < -180) {
                    turretSetpointInDegrees = interimTurretAzimuth + 360.0;  
                } else {
                    turretSetpointInDegrees = interimTurretAzimuth;                     
                }
                if(turretSetpointInDegrees > TurretConstants.kPhysicalTurretRotationLimit) {
                    turretSetpointInDegrees = TurretConstants.kPhysicalTurretRotationLimit;
                } else if (turretSetpointInDegrees < -TurretConstants.kPhysicalTurretRotationLimit) {
                    turretSetpointInDegrees = -TurretConstants.kPhysicalTurretRotationLimit;
                }
            } 
            double turretMotorSpeed = mPid.updateRotation(turretAzimuth, turretSetpointInDegrees);
            mTurret.set(ControlMode.PercentOutput, turretMotorSpeed);                
        }
    }

    /**
     * Manually Drives the Turret only when Auto Targeting is disabled
     * <p> 
     * Scaled down by kManualControlScaleFactor
     * 
     * @param pow the input power from [-1,1] to be scaled down to drive the turret
     */
    public void manualControl(double pow) {
        if(!mAutoTargetingEnabled) {
            mTurret.set(ControlMode.PercentOutput, pow * TurretConstants.kManualControlScaleFactor);
        }
    }

    /**
     * Is Turret Aimed at Target AND does the ball have clearance with this angle
     * 
     * @return whether to take the shot
     */
    public synchronized boolean isAimedAtTarget() {
        double tx = LimelightManagerSubsystem.getInstance().getShooterLimelight().getTx();
        return Math.abs(tx) < TurretConstants.kTurretEpsilon 
                && TurretConstants.kTargetWidth[mSelectedTarget.ordinal()] * Math.cos(Math.abs(DriveSubsystem.getInstance().getHeading() + getTurretAngle() + tx)) - TurretConstants.kPowerCellClearance > TurretConstants.kPowerCellDiameterInches;
    }

    /**
     * Returns the Robot-Relative Turret Angle
     * 
     * @return Robot-Relative Turret Angle
     */
    public double getTurretAngle() {
        return mEncoder.getDistance();
    }

    /**
     * Gets if Turret Auto-Targeting is Enabled
     * 
     * @return <i> true </i> if the turret automatically seeking the target is enabled
     */
    public boolean isAutoTargetingEnabled() {
        return mAutoTargetingEnabled;
    }

    /**
     * Toggles whether the turret is allowed to automatically seek the target or not
     */
    public void toggleAutoTargetingEnabled() {
        if(mAutoTargetingEnabled) {
            mAutoTargetingEnabled = false;
        } else {
            mAutoTargetingEnabled = true;
        }
    }

    /**
     * Sets the Target of the shooter to either the High or Low Target
     * 
     * @param target the target for the Shooter to target
     */
    public void setTarget(ShootingTarget target) {
        mSelectedTarget = target;
    }

    /**
     * The Target Options of the Shooter
     */
    public enum ShootingTarget implements Sendable {
        eHighTarget(), eLowTarget();

        private ShootingTarget() {}

        @Override
        public void initSendable(SendableBuilder builder) {}
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putData("Shooting Target", mSelectedTarget);
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putBoolean("Aimed at Target", isAimedAtTarget());
        SmartDashboard.putBoolean("Auto Targeting Enabled", mAutoTargetingEnabled);
    }

    @Override
    public void runTest() {
        // TODO Auto-generated method stub
        
    }

    /**
     * @return TurretSubsystem Singleton Instance
     */
    public synchronized static TurretSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new TurretSubsystem();
        }
        return mInstance;
    }
}
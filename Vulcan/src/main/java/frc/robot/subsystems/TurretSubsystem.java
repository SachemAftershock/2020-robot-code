package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.PID;
import frc.robot.Constants.SuperstructureConstants.TurretConstants;

public class TurretSubsystem extends SubsystemBase implements SubsystemInterface {

    private static TurretSubsystem mInstance;

    private WPI_VictorSPX mTurret;
    private DutyCycleEncoder mEncoder;
    private PID mPid;

    private boolean mAutoTargetingEnabled;
    private ShootingTarget mSelectedTarget;

    public TurretSubsystem() {
        mTurret = new WPI_VictorSPX(TurretConstants.kTurretMotorId);
        mTurret.setNeutralMode(NeutralMode.Brake);

        mEncoder = new DutyCycleEncoder(new DigitalInput(TurretConstants.kTurretEncoderDioId));
        mEncoder.setDistancePerRotation(TurretConstants.kTurretDegreesPerEncoderRotation);
        mEncoder.reset();

        mPid = new PID(TurretConstants.kTurretEpsilon);
        mPid.start(TurretConstants.kGains);

        mAutoTargetingEnabled = true; //TODO: Maybe want a way to disable this?

        mSelectedTarget = ShootingTarget.eHighTarget; //TODO: Add Buttons for high and low
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        if(DriverStation.getInstance().isAutonomous() || mAutoTargetingEnabled) {
            final double tx = Limelight.getTx();
            final double robotAziumth = DriveSubsystem.getInstance().getHeading();  // TODO: make sure zero means heading is downfield.
            final double turretAzimuth = mEncoder.getDistance();  // Turret Azimuth in -180..180, zero is turrent inline robot forward.
            double turretSetpointInDegrees = 0.0; 
            // -180..180deg where 0deg is turret inline looking forward on robot.
            // Simply change this setpoint at any time to redirect the turret.

            final boolean isLimelightAimedDownfield = ((robotAziumth + turretAzimuth) > -90.0) && ((robotAziumth + turretAzimuth) < 90.0);

            if ((tx >= Limelight.kDefaultTx) || !isLimelightAimedDownfield) {
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
            } 
            double turretMotorSpeed = mPid.updateRotation(turretAzimuth, turretSetpointInDegrees);
            mTurret.set(ControlMode.PercentOutput, turretMotorSpeed);                
        }
    }

    public void manualControl(double pow) {
        mTurret.set(ControlMode.PercentOutput, pow * TurretConstants.kManualControlScaleFactor);
    }

    /**
     * Is Turret Aimed at Target AND does the ball have clearance with this angle
     * @return whether to take the shot
     */
    public boolean isAimedAtTarget() {
        double tx = Limelight.getTx();
        return Math.abs(tx) < TurretConstants.kTurretEpsilon 
                && TurretConstants.kTargetWidth[mSelectedTarget.ordinal()] * Math.cos(Math.abs(DriveSubsystem.getInstance().getHeading() + getTurretAngle() + tx)) - TurretConstants.kPowerCellClearance > TurretConstants.kPowerCellDiameterInches;
    }

    /**
     * Returns the Robot-Relative Turret Angle
     * @return Robot-Relative Turret Angle
     */
    public double getTurretAngle() {
        return mEncoder.getDistance();
    }

    enum ShootingTarget {
        eHighTarget, eLowTarget
    }

    public synchronized static TurretSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new TurretSubsystem();
        }
        return mInstance;
    }
}
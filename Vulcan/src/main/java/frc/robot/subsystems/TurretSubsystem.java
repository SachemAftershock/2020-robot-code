package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.PID;

public class TurretSubsystem extends SubsystemBase {

    private static TurretSubsystem mInstance;

    private WPI_VictorSPX mTurret;
    private DutyCycleEncoder mEncoder;
    private PID mPid;

    private boolean mAutoTargetingEnabled;

    private final double kManualControlScaleFactor = 0.25;
    private final double[] mGains = {0.0, 0.0, 0.0};
    private final double kTurretEpsilon = 4.0; //TODO: Find the right value
    private final double kTurretDegreesPerEncoderRotation = 45.0; // 8 rot == 360 deg

    public TurretSubsystem() {
        mTurret = new WPI_VictorSPX(Constants.kTurretMotorId);
        mTurret.setNeutralMode(NeutralMode.Brake);

        mEncoder = new DutyCycleEncoder(new DigitalInput(Constants.kTurretEncoderDioId));
        mEncoder.setDistancePerRotation(kTurretDegreesPerEncoderRotation);
        mEncoder.reset();

        mPid = new PID(kTurretEpsilon);
        mPid.start(mGains);

        mAutoTargetingEnabled = true; //TODO: Maybe want a way to disable this?
    }
    //TODO: Guard against over rotating

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
                turretSetpointInDegrees = turretAzimuth + tx;  
            } 
            double turretMotorSpeed = mPid.updateRotation(turretAzimuth, turretSetpointInDegrees);
            mTurret.set(ControlMode.PercentOutput, turretMotorSpeed);                
        }
    }

    public void manualControl(double pow) {
        mTurret.set(ControlMode.PercentOutput, pow * kManualControlScaleFactor);
    }

    public boolean isAimedAtTarget() {
        return Math.abs(Limelight.getTx()) < kTurretEpsilon;
    }

    public synchronized static TurretSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new TurretSubsystem();
        }
        return mInstance;
    }
}
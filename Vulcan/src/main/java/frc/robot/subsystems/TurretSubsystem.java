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

    private boolean mAutoTargetingEnabled, mNewTargetFound, mCurrentTargetExists;

    private final double kManualControlScaleFactor = 0.25;
    private final double[] mGains = {0.0, 0.0, 0.0};
    private final double kTurretEpsilon = 4.0; //TODO: Find the right value
    private final double kTurretDegreesPerEncoderRotation = 45.0; // 8 rot == 360 deg
    private final double kRequiredHeadingToTarget = 180.0; //If the start position is angled in any way this will have to change

    public TurretSubsystem() {
        mTurret = new WPI_VictorSPX(Constants.kTurretMotorId);
        mTurret.setNeutralMode(NeutralMode.Brake);

        mEncoder = new DutyCycleEncoder(new DigitalInput(Constants.kTurretEncoderDioId));
        mEncoder.setDistancePerRotation(kTurretDegreesPerEncoderRotation);
        mEncoder.reset();

        mPid = new PID(kTurretEpsilon);

        mAutoTargetingEnabled = true;
        mNewTargetFound = false;
        mCurrentTargetExists = false;
    }
    //TODO: Guard against over rotating

    @Override
    public void periodic() {
        if(DriverStation.getInstance().isAutonomous() || mAutoTargetingEnabled) {
            final double tx = Limelight.getTx();
            final boolean isTargetFound = tx < 999.0;

            if(isTargetFound && !mCurrentTargetExists) {
                mNewTargetFound = true;
            }
            if(isTargetFound) {
                if(mNewTargetFound) {
                    mPid.start(mGains);
                    mNewTargetFound = false;
                }
                mCurrentTargetExists = true;
                final double output = mPid.update(tx, 0.0);
                mTurret.set(ControlMode.PercentOutput, output);
            } else if(mAutoTargetingEnabled) {
                mNewTargetFound = false;
                mCurrentTargetExists = false;
                final double heading = DriveSubsystem.getInstance().getHeading();
                final double pos = mEncoder.getDistance();
                
            }
        }
    }

    public void manualControl(double pow) {
        mTurret.set(ControlMode.PercentOutput, pow * kManualControlScaleFactor);
    }

    public synchronized static TurretSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new TurretSubsystem();
        }
        return mInstance;
    }
}
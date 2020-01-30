package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Lidar;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem mInstance;
    
    private final CANSparkMax mShooter;
    private final WPI_VictorSPX mFeeder;
    private final DoubleSolenoid mElevationSolenoid;
    private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;
    private final double[] mGains = {0.0, 0.0, 0.0, 0.0, 0.0}; //P I D Iz FF
    private final double kTargetRPM = 6000.0; //TODO: Get this value
    private final double kShooterSpeedEpsilon = 50.0;
    private final double kFeederSpeed = 0.65;

    public ShooterSubsystem() {
        mShooter = new CANSparkMax(Constants.kLauncherMotorId, MotorType.kBrushless);
        mShooter.setIdleMode(IdleMode.kCoast); //Brake mode might be really bad

        mFeeder = new WPI_VictorSPX(Constants.kFeederMotorId);

        mLidar = new Lidar(new DigitalInput(Constants.kLidarId));

        mShooterEncoder = mShooter.getEncoder();

        mShooterPid = new CANPIDController(mShooter);
        mShooterPid.setP(mGains[0], Constants.kPidId);
        mShooterPid.setI(mGains[1], Constants.kPidId);
        mShooterPid.setD(mGains[2], Constants.kPidId);
        mShooterPid.setIZone(mGains[3], Constants.kPidId);
        mShooterPid.setFF(mGains[4], Constants.kPidId);
        mShooterPid.setOutputRange(-1.0, 1.0);

        mElevationSolenoid = new DoubleSolenoid(Constants.kPcmId, Constants.kElevationForwardId, Constants.kElevationReverseId);
    }

    @Override
    public void periodic() {

    }

    public void reachTargetRPM() {
        mShooterPid.setReference(kTargetRPM, ControlType.kVelocity, Constants.kPidId);
    }
    public void stopShooter() {
        mShooter.set(0.0);
    }

    public void runFeeder() {
        mFeeder.set(ControlMode.PercentOutput, kFeederSpeed);
    }
    public void stopFeeder() {
        mFeeder.set(ControlMode.PercentOutput, 0.0);
    }

	public void toggleElevation() {
        switch (mElevationSolenoid.get()) {
            case kForward: 
                mElevationSolenoid.set(Value.kReverse);
                break;
            case kReverse: 
                mElevationSolenoid.set(Value.kForward);
                break;
            default:
                mElevationSolenoid.set(Value.kReverse);
                DriverStation.reportError("ERROR: SHOOTER ELEVATION VALUE INVALID", false);
                break;
        }
    }
    
    public static ShooterSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ShooterSubsystem();
        }
        return mInstance;
    }
}


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lidar;
import frc.robot.Constants.SuperstructureConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ShooterSubsystem mInstance;
    
    private final CANSparkMax mShooter;
    private final WPI_TalonSRX mFeeder;
    private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;

    private double mTargetRPM;

    public ShooterSubsystem() {
        mShooter = new CANSparkMax(ShooterConstants.kLauncherMotorId, MotorType.kBrushless);
        mShooter.setIdleMode(IdleMode.kCoast); //Brake mode might be really bad

        mFeeder = new WPI_TalonSRX(ShooterConstants.kFeederMotorId);

        mLidar = new Lidar(new DigitalInput(ShooterConstants.kLidarId));

        mShooterEncoder = mShooter.getEncoder();

        mShooterPid = new CANPIDController(mShooter);
        mShooterPid.setP(ShooterConstants.kGains[0], ShooterConstants.kPidId);
        mShooterPid.setI(ShooterConstants.kGains[1], ShooterConstants.kPidId);
        mShooterPid.setD(ShooterConstants.kGains[2], ShooterConstants.kPidId);
        mShooterPid.setIZone(ShooterConstants.kGains[3], ShooterConstants.kPidId);
        mShooterPid.setFF(ShooterConstants.kGains[4], ShooterConstants.kPidId);
        mShooterPid.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void init() {
    }
    
    //TODO: Rumble / LEDs when at Target RPM
    //TODO: Add code for when Target in FOV: Use the predictive lookup, when falls out of view,
    // after a timeout, goes to the median Table RPM. Defaults to median value when target is not found
    public void reachCalculatedTargetRPM() {
        mTargetRPM = ShooterConstants.kShooterPolynomial.predict(mLidar.getDistanceIn() / 12.0);
        mShooterPid.setReference(mTargetRPM, ControlType.kVelocity, ShooterConstants.kPidId);
    }

    public void stopShooter() {
        mShooter.set(0.0);
    }

    public boolean isAtTargetRPM() {
        return Math.abs(mShooterEncoder.getVelocity() - mTargetRPM) < ShooterConstants.kShooterSpeedEpsilon;
    }

    public void startFeeder() {
        mFeeder.set(ControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        mFeeder.set(ControlMode.PercentOutput, 0.0);
    }
    
    public synchronized static ShooterSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ShooterSubsystem();
        }
        return mInstance;
    }
}


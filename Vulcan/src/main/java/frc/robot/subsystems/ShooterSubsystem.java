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
import frc.robot.Constants;
import frc.robot.Lidar;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem mInstance;
    
    private final CANSparkMax mShooter;
    private final WPI_TalonSRX mFeeder;
    private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;
    private final double[] mGains = {0.0, 0.0, 0.0, 0.0, 0.0}; //P I D Iz FF
    private final double kShooterSpeedEpsilon = 50.0;
    private final double kFeederSpeed = 0.65;

    private double mTargetRPM = 6000.0; //TODO: Get a way to calculate this value 

    public ShooterSubsystem() {
        mShooter = new CANSparkMax(Constants.kLauncherMotorId, MotorType.kBrushless);
        mShooter.setIdleMode(IdleMode.kCoast); //Brake mode might be really bad

        mFeeder = new WPI_TalonSRX(Constants.kFeederMotorId);

        mLidar = new Lidar(new DigitalInput(Constants.kLidarId));

        mShooterEncoder = mShooter.getEncoder();

        mShooterPid = new CANPIDController(mShooter);
        mShooterPid.setP(mGains[0], Constants.kPidId);
        mShooterPid.setI(mGains[1], Constants.kPidId);
        mShooterPid.setD(mGains[2], Constants.kPidId);
        mShooterPid.setIZone(mGains[3], Constants.kPidId);
        mShooterPid.setFF(mGains[4], Constants.kPidId);
        mShooterPid.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void periodic() {

    }

    public void reachCalculatedTargetRPM() {
        mShooterPid.setReference(mTargetRPM, ControlType.kVelocity, Constants.kPidId);
    }

    public void stopShooter() {
        mShooter.set(0.0);
    }

    public boolean isAtTargetRPM() {
        return Math.abs(mShooterEncoder.getVelocity() - mTargetRPM) < kShooterSpeedEpsilon;
    }

    public void startFeeder() {
        mFeeder.set(ControlMode.PercentOutput, kFeederSpeed);
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


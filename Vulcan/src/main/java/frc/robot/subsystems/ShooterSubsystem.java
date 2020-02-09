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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControllerRumble;
import frc.robot.Lidar;
import frc.robot.LimelightManager;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperstructureConstants.ShooterConstants;

/**
 * Shooter Subsystem
 * @author Shreyas Prasad
 */
public class ShooterSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ShooterSubsystem mInstance;
    
    private final CANSparkMax mShooter;
    private final WPI_TalonSRX mFeeder;
    private final Lidar mLidar;

    private final CANEncoder mShooterEncoder;
    private final CANPIDController mShooterPid;

    private double mTargetRPM;

    private int mRumbleDelayCounter, mTargetFalloutDelayCounter;

    private ShooterSubsystem() {
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

        mRumbleDelayCounter = 0;
        mTargetFalloutDelayCounter = 0;
    }

    @Override
    public void init() {
    }
    
    //TODO: LEDs when at Target RPM
    public void reachCalculatedTargetRPM() {
        if(LimelightManager.getInstance().getShooterLimelight().isTarget()) {
            mTargetRPM = ShooterConstants.kShooterPolynomial.predict(mLidar.getDistanceIn() / 12.0);
            mTargetFalloutDelayCounter = 0;
        } else if(mTargetFalloutDelayCounter > 2500 / 20) {
            mTargetRPM = ShooterConstants.kDistanceFeetToRpmLUT[(int) ShooterConstants.kDistanceFeetToRpmLUT.length / 2][1];
            mTargetFalloutDelayCounter = 0;
        } else {
            mTargetFalloutDelayCounter++;
        }
        mShooterPid.setReference(mTargetRPM, ControlType.kVelocity, ShooterConstants.kPidId);
    }

    public void stopShooter() {
        mShooter.set(0.0);
    }

    public boolean isAtTargetRPM() {
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

    public void startFeeder() {
        mFeeder.set(ControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
    }

    public void stopFeeder() {
        mFeeder.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Velocity", mShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Target Velocity", mTargetRPM);
        SmartDashboard.putNumber("Lidar Distance Ft", mLidar.getDistanceIn() / 12.0);
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


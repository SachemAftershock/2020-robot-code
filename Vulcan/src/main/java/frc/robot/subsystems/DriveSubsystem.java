package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Util;

public class DriveSubsystem extends SubsystemBase {

    private static DriveSubsystem mInstance;

    private final CANSparkMax mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC;
    private final SpeedControllerGroup mDriveGroupPort;
    private final CANEncoder mPortEncoder;
    private final CANSparkMax mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC;
    private final SpeedControllerGroup mDriveGroupStarboard;
    private final CANEncoder mStarboardEncoder;
    private final DifferentialDrive mDifferentialDrive;
    private final DifferentialDriveKinematics mKinematics;
    private final DifferentialDriveOdometry mOdometry;
    private final DoubleSolenoid mGearShifter;
    private final PID mPortPid, mStarboardPid, mRotatePid;
    private final AHRS mNavx;

    private Pose2d mPose;

    private double mLeft, mRight, mLeftTarget, mRightTarget, mRotateSetpoint;
    private final double kSlowdownProportion = 0.6;
    private final double kDriveEpsilon = 2.0; //TODO: Find a good value for this
    private final double kRotateEpsilon = 3.0;
    private final double[] mLinearGains = {0.0, 0.0, 0.0};
    private final double[] mRotationalGains = {0.0, 0.0, 0.0};
    private final double kRampRateToMaxSpeed = 1.5;

    private double mSelectedMaxSpeedProportion;
    private final double kRegularMaxSpeed = 1.0;
    private final double kPrecisionMaxSpeed = 0.5; //TODO: Find a good value for this

    public DriveSubsystem() {

        mDriveMotorPortA = new CANSparkMax(Constants.kDriveMotorPortAId, MotorType.kBrushless);
        mDriveMotorPortA.setInverted(false);
        mDriveMotorPortA.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorPortB = new CANSparkMax(Constants.kDriveMotorPortBId, MotorType.kBrushless);
        mDriveMotorPortB.setInverted(false);
        mDriveMotorPortB.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorPortC = new CANSparkMax(Constants.kDriveMotorPortCId, MotorType.kBrushless);
        mDriveMotorPortC.setInverted(false);
        mDriveMotorPortC.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveGroupPort = new SpeedControllerGroup(mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC);
        addChild("Speed Controller Group Port Side",mDriveGroupPort);
                
        mDriveMotorStarboardA = new CANSparkMax(Constants.kDriveMotorStarboardAId, MotorType.kBrushless);
        mDriveMotorStarboardA.setInverted(true);
        mDriveMotorStarboardA.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorStarboardB = new CANSparkMax(Constants.kDriveMotorStarboardBId, MotorType.kBrushless);
        mDriveMotorStarboardB.setInverted(true);
        mDriveMotorStarboardB.setOpenLoopRampRate(kRampRateToMaxSpeed);

        mDriveMotorStarboardC = new CANSparkMax(Constants.kDriveMotorStarboardCId, MotorType.kBrushless);
        mDriveMotorStarboardC.setInverted(true);
        mDriveMotorStarboardC.setOpenLoopRampRate(kRampRateToMaxSpeed);

        mDriveGroupStarboard = new SpeedControllerGroup(mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC  );
                
        mDifferentialDrive = new DifferentialDrive(mDriveGroupPort, mDriveGroupStarboard);
        addChild("Differential Drive", mDifferentialDrive);
        mDifferentialDrive.setSafetyEnabled(true);
        mDifferentialDrive.setExpiration(0.1);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);

        mPortEncoder = mDriveMotorPortA.getEncoder();
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();
        resetEncoders();

        mKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);

        //TODO: Find out if I need to change the range of degrees the Navx gives, currently [-180, 180]
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(mNavx.getYaw()),
        new Pose2d(0, 0, new Rotation2d()));
        //TODO: the auto I choose is gonna have to change the above x, y, and theta values
                
        mGearShifter = new DoubleSolenoid(Constants.kPcmId, Constants.kGearShiftForwardId, Constants.kGearShiftReverseId);
        addChild("Gear Shift Port Double Solenoid", mGearShifter);    

        mPortPid = new PID(kDriveEpsilon);
        mStarboardPid = new PID(kDriveEpsilon);

        mRotatePid = new PID(kRotateEpsilon);

        mSelectedMaxSpeedProportion = kRegularMaxSpeed;

        mLeft = 0.0;
        mRight = 0.0;
        mLeftTarget = 0.0;
        mRightTarget = 0.0;
        mRotateSetpoint = 0.0;
    }

    @Override
    public void periodic() {
        //navx getYaw is negative because Navx gives clockwise as positive values, WPILIB uses opposite
        mPose = mOdometry.update(Rotation2d.fromDegrees(-mNavx.getYaw()), getPortEncoderDistance(), getStarboardEncoderDistance());
    }
    
    public void manualDrive(double pow, double rot) {
        mDifferentialDrive.curvatureDrive(pow * mSelectedMaxSpeedProportion, rot * mSelectedMaxSpeedProportion, true);
    }

    public void startAutoDrive(double setpoint) {
        mLeftTarget = mPortEncoder.getPosition() + getEncoderCount(setpoint); //Idk if this is right at all
        mRightTarget = mStarboardEncoder.getPosition() + getEncoderCount(setpoint);
        mPortPid.start(mLinearGains);
        mStarboardPid.start(mLinearGains);
    }

    public void runAutoDrive() {
        double scaleFactor = CollisionAvoidanceSubsystem.getInstance().getSlowdownScaleFactor();
        if(scaleFactor < 1.0) {
            mLeft *= scaleFactor;
            mRight *= scaleFactor;
            mPortPid.pausePID();
            mStarboardPid.pausePID();
        } else {
            if(mPortPid.isPaused() || mStarboardPid.isPaused()) {
                mPortPid.resumePID();
                mStarboardPid.resumePID();
            }
            mLeft = mPortPid.update(mPortEncoder.getPosition(), mLeftTarget);
            mRight = mStarboardPid.update(mStarboardEncoder.getPosition(), mRightTarget);
        }
        mDifferentialDrive.tankDrive(mLeft, mRight);
        //If when driving staright, the robot is turning, add gyro-based correction
    }

    public boolean linearDriveTargetReached() {
        return Math.abs(mPortPid.getError()) <= kDriveEpsilon &&
             Math.abs(mStarboardPid.getError()) <= kDriveEpsilon;
    }

    public void startAutoRotate(double theta) {
        mRotateSetpoint = theta;
        mRotatePid.start(mRotationalGains);
    }

    public void runAutoRotate() {
        double theta = Util.normalizeAngle(mRotateSetpoint);
        double output = mRotatePid.updateRotation(mNavx.getYaw(), theta);
        mDifferentialDrive.tankDrive(output, -output);
    }

    public boolean rotateTargetReached() {
        return Math.abs(mRotatePid.getError()) <= kRotateEpsilon;
    }

    public void togglePrecisionDriving() {
        if(mSelectedMaxSpeedProportion == kRegularMaxSpeed) {
            mSelectedMaxSpeedProportion = kPrecisionMaxSpeed;
        } else {
            mSelectedMaxSpeedProportion = kRegularMaxSpeed;
        }
    }

    public void toggleDrivebaseGearing(){
        switch (mGearShifter.get()) {
            case kForward : 
                mGearShifter.set(Value.kReverse);
                break;
            case kReverse : 
                mGearShifter.set(Value.kForward);
                break;
            default :
                mGearShifter.set(Value.kReverse);
                DriverStation.reportError("ERROR: GEAR SHIFT VALUE INVALID", false);
                break;
        }
    }

    private double getPortEncoderRotations() {
        return mPortEncoder.getPosition() / Constants.kDriveEncoderPPR;
    }

    private double getStarboardEncoderRotations() {
        return mStarboardEncoder.getPosition() / Constants.kDriveEncoderPPR;
    }

    private double rotationsToInches(double rotations) {
        return rotations * Constants.kWheelCircumference;
    }

    private double getPortEncoderDistance() {
        return rotationsToInches(getPortEncoderRotations());
    }

    private double getStarboardEncoderDistance() {
        return rotationsToInches(getStarboardEncoderRotations());
    }

    private double getEncoderCount(double distance) {
        return distance / Constants.kWheelCircumference * Constants.kDriveEncoderPPR;
    }

    private double getPortRPM() {
        return mPortEncoder.getVelocity();
    }

    private double getStarboardRPM() {
        return mStarboardEncoder.getVelocity();
    }

    private double getPortSpeedMetersPerSecond() {
        return Util.convertRPMToRadiansPerSecond(getPortRPM()) * Constants.kWheelRadiusMeters;
    }

    private double getStarboardSpeedMetersPerSecond() {
        return Util.convertRPMToRadiansPerSecond(getStarboardRPM()) * Constants.kWheelRadiusMeters;
    }

    public void resetEncoders() {
        mPortEncoder.setPosition(0.0);
        mStarboardEncoder.setPosition(0.0);
    }

    public static DriveSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new DriveSubsystem();
        }
        return mInstance;
    }
}


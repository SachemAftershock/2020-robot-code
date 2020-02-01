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
import edu.wpi.first.wpilibj.util.Units;
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
    private double mLeftSpeed, mRightSpeed, mLeftTarget, mRightTarget, mRotateSetpoint;
    private boolean mIsHighGear;

    //TODO: Find a good value for all constants below
    private final double kDriveEpsilon = 2.0; 
    private final double kRotateEpsilon = 3.0;
    private final double[] mLinearGains = {0.0, 0.0, 0.0};
    private final double[] mRotationalGains = {0.0, 0.0, 0.0};
    private final double kRampRateToMaxSpeed = 1.0;

    private double mSelectedMaxSpeedProportion;
    private final double kRegularMaxSpeed = 1.0;
    private final double kPrecisionMaxSpeed = 0.5;

    private final double kWheelDiameter = 6;
    private final double kWheelCircumference = kWheelDiameter * Math.PI;
    private final double kDriveEncoderPPR = 42.0;
    private final double kWheelRadiusMeters = Units.inchesToMeters(kWheelDiameter / 2.0);
    private final double kTrackWidthMeters = Units.inchesToMeters(19.125); //Distance in between wheels
    private final double kLowGearRatio = 1.0;
    private final double kHighGearRatio = 1.0;
    
    //Robot Characterization Variables Below
    private final double ksVolts = 1.0;
    private final double kvVoltSecondsPerMeter = 1.0;
    private final double kaVoltSecondsSquaredPerMeter = 1.0;
    //Tuned from above vales
    private final double kP = 1.0;

    public DriveSubsystem() {
        //Differential Drive Class inverts right side
        mDriveMotorPortA = new CANSparkMax(Constants.kDriveMotorPortAId, MotorType.kBrushless);
        //mDriveMotorPortA.setInverted(false);
        mDriveMotorPortA.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorPortB = new CANSparkMax(Constants.kDriveMotorPortBId, MotorType.kBrushless);
        //mDriveMotorPortB.setInverted(false);
        mDriveMotorPortB.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorPortC = new CANSparkMax(Constants.kDriveMotorPortCId, MotorType.kBrushless);
        //mDriveMotorPortC.setInverted(false);
        mDriveMotorPortC.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveGroupPort = new SpeedControllerGroup(mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC);
        addChild("Speed Controller Group Port Side",mDriveGroupPort);
                
        mDriveMotorStarboardA = new CANSparkMax(Constants.kDriveMotorStarboardAId, MotorType.kBrushless);
        //mDriveMotorStarboardA.setInverted(true);
        mDriveMotorStarboardA.setOpenLoopRampRate(kRampRateToMaxSpeed);
                
        mDriveMotorStarboardB = new CANSparkMax(Constants.kDriveMotorStarboardBId, MotorType.kBrushless);
        //mDriveMotorStarboardB.setInverted(true);
        mDriveMotorStarboardB.setOpenLoopRampRate(kRampRateToMaxSpeed);

        mDriveMotorStarboardC = new CANSparkMax(Constants.kDriveMotorStarboardCId, MotorType.kBrushless);
        //mDriveMotorStarboardC.setInverted(true);
        mDriveMotorStarboardC.setOpenLoopRampRate(kRampRateToMaxSpeed);

        mDriveGroupStarboard = new SpeedControllerGroup(mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC  );
                
        mDifferentialDrive = new DifferentialDrive(mDriveGroupPort, mDriveGroupStarboard);
        addChild("Differential Drive", mDifferentialDrive);
        mDifferentialDrive.setSafetyEnabled(true);
        mDifferentialDrive.setExpiration(0.1);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);

        mPortEncoder = mDriveMotorPortA.getEncoder();
        //Convert RPM to Rad/s
        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * kLowGearRatio);
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * kLowGearRatio);
        resetEncoders();

        mKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

        //TODO: Find out if I need to change the range of degrees the Navx gives, currently [-180, 180]
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(mNavx.getYaw()),
        new Pose2d(0, 0, new Rotation2d()));
        //TODO: the auto I choose is gonna have to change the above x, y, and theta values
                
        mGearShifter = new DoubleSolenoid(Constants.kPcmAId, Constants.kGearShiftForwardId, Constants.kGearShiftReverseId);
        addChild("Gear Shift Port Double Solenoid", mGearShifter);    
        mIsHighGear = false;
        mGearShifter.set(Value.kReverse); // TODO: Find out which side corresponds to which gearing

        mPortPid = new PID(kDriveEpsilon);
        mStarboardPid = new PID(kDriveEpsilon);

        mRotatePid = new PID(kRotateEpsilon);

        mSelectedMaxSpeedProportion = kRegularMaxSpeed;

        mLeftSpeed = 0.0;
        mRightSpeed = 0.0;
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
            mLeftSpeed *= scaleFactor;
            mRightSpeed *= scaleFactor;
            mPortPid.pausePID();
            mStarboardPid.pausePID();
        } else {
            if(mPortPid.isPaused() || mStarboardPid.isPaused()) {
                mPortPid.resumePID();
                mStarboardPid.resumePID();
            }
            mLeftSpeed = mPortPid.update(mPortEncoder.getPosition(), mLeftTarget);
            mRightSpeed = mStarboardPid.update(mStarboardEncoder.getPosition(), mRightTarget);
        }
        mDifferentialDrive.tankDrive(mLeftSpeed, mRightSpeed);
        //If when driving straight, the robot is turning, add gyro-based correction
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
                mIsHighGear = false;
                break;
            case kReverse : 
                mGearShifter.set(Value.kForward);
                mIsHighGear = true;
                break;
            default :
                mGearShifter.set(Value.kReverse);
                mIsHighGear = false;
                DriverStation.reportError("ERROR: GEAR SHIFT VALUE INVALID", false);
                break;
        }
    }

    private double getPortEncoderRotations() {
        return mPortEncoder.getPosition() / kDriveEncoderPPR;
    }

    private double getStarboardEncoderRotations() {
        return mStarboardEncoder.getPosition() / kDriveEncoderPPR;
    }

    private double rotationsToInches(double rotations) {
        return rotations * kWheelCircumference;
    }

    private double getPortEncoderDistance() {
        return rotationsToInches(getPortEncoderRotations());
    }

    private double getStarboardEncoderDistance() {
        return rotationsToInches(getStarboardEncoderRotations());
    }

    private double getEncoderCount(double distance) {
        return distance / kWheelCircumference * kDriveEncoderPPR;
    }

    /**
     * @return Port Side Angular Velocity in Rad/s
     */
    private double getPortAngularVelocity() {
        return mPortEncoder.getVelocity();
    }
    /**
     * @return Starboard Side Angular Velocity in Rad/s
     */
    private double getStarboardAngularVelocity() {
        return mStarboardEncoder.getVelocity();
    }

    private double getPortSpeedMetersPerSecond() {
        return getPortAngularVelocity() * kWheelRadiusMeters;
    }

    private double getStarboardSpeedMetersPerSecond() {
        return getStarboardAngularVelocity() * kWheelRadiusMeters;
    }

    public void resetEncoders() {
        mPortEncoder.setPosition(0.0);
        mStarboardEncoder.setPosition(0.0);
    }

    public double getHeading() {
        return mNavx.getYaw();
    }

    public synchronized static DriveSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new DriveSubsystem();
        }
        return mInstance;
    }
}
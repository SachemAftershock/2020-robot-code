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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PID;
import frc.robot.Util;
import frc.robot.Constants.DriveConstants;

/**
 * Drivebase Subsystem
 * @author Shreyas Prasad
 */
public class DriveSubsystem extends SubsystemBase implements SubsystemInterface {

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

    private double mSelectedMaxSpeedProportion;
    private boolean mIsPrecisionMode;

    private DriveSubsystem() {
        //Differential Drive Class inverts right side
        mDriveMotorPortA = new CANSparkMax(DriveConstants.kDriveMotorPortAId, MotorType.kBrushless);
        //mDriveMotorPortA.setInverted(false);
        mDriveMotorPortA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorPortB = new CANSparkMax(DriveConstants.kDriveMotorPortBId, MotorType.kBrushless);
        //mDriveMotorPortB.setInverted(false);
        mDriveMotorPortB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorPortC = new CANSparkMax(DriveConstants.kDriveMotorPortCId, MotorType.kBrushless);
        //mDriveMotorPortC.setInverted(false);
        mDriveMotorPortC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveGroupPort = new SpeedControllerGroup(mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC);
        addChild("Speed Controller Group Port Side",mDriveGroupPort);
                
        mDriveMotorStarboardA = new CANSparkMax(DriveConstants.kDriveMotorStarboardAId, MotorType.kBrushless);
        //mDriveMotorStarboardA.setInverted(true);
        mDriveMotorStarboardA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorStarboardB = new CANSparkMax(DriveConstants.kDriveMotorStarboardBId, MotorType.kBrushless);
        //mDriveMotorStarboardB.setInverted(true);
        mDriveMotorStarboardB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);

        mDriveMotorStarboardC = new CANSparkMax(DriveConstants.kDriveMotorStarboardCId, MotorType.kBrushless);
        //mDriveMotorStarboardC.setInverted(true);
        mDriveMotorStarboardC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);

        mDriveGroupStarboard = new SpeedControllerGroup(mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC  );
                
        mDifferentialDrive = new DifferentialDrive(mDriveGroupPort, mDriveGroupStarboard);
        addChild("Differential Drive", mDifferentialDrive);
        mDifferentialDrive.setSafetyEnabled(true);
        mDifferentialDrive.setExpiration(0.1);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);

        //TODO: Need to check over all encoder related calculations, not sure I trust them
        mPortEncoder = mDriveMotorPortA.getEncoder();
        //Convert RPM to Rad/s
        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);

        mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

        //TODO: Find out if I need to change the range of degrees the Navx gives, currently [-180, 180]
        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()),
        new Pose2d(0, 0, new Rotation2d()));
        //TODO: the auto I choose is gonna have to change the above x, y, and theta values
                
        mGearShifter = new DoubleSolenoid(Constants.kPcmAId, DriveConstants.kGearShiftForwardId, DriveConstants.kGearShiftReverseId);
        addChild("Gear Shift Port Double Solenoid", mGearShifter);    
        mGearShifter.set(Value.kReverse); // TODO: Find out which side corresponds to which gearing

        mPortPid = new PID(DriveConstants.kDriveEpsilon);
        mStarboardPid = new PID(DriveConstants.kDriveEpsilon);

        mRotatePid = new PID(DriveConstants.kRotateEpsilon);

        mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
        mIsPrecisionMode = false;

        mLeftSpeed = 0.0;
        mRightSpeed = 0.0;
        mLeftTarget = 0.0;
        mRightTarget = 0.0;
        mRotateSetpoint = 0.0;
    }

    @Override
    public void init() {
        resetEncoders();
        mNavx.zeroYaw();
    }

    @Override
    public void periodic() {
        mPose = mOdometry.update(Rotation2d.fromDegrees(getHeading()), getPortEncoderDistance(), getStarboardEncoderDistance());
    }
    
    public void manualDrive(double pow, double rot) {
        mDifferentialDrive.curvatureDrive(pow * mSelectedMaxSpeedProportion, rot * mSelectedMaxSpeedProportion, true);
    }

    /**
     * Set target destination for Autonomous Linear Drive
     * @param setpoint target destination to drive forward to
     */
    public void startAutoDrive(double setpoint) {
        mLeftTarget = mPortEncoder.getPosition() + getEncoderCount(setpoint);
        mRightTarget = mStarboardEncoder.getPosition() + getEncoderCount(setpoint);
        mPortPid.start(DriveConstants.kLinearGains);
        mStarboardPid.start(DriveConstants.kLinearGains);
    }

    /**
     * Process Autonomous Linear Drive
     * Limited by Collision Avoidance if collision imminent 
     */
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
        return Math.abs(mPortPid.getError()) <= DriveConstants.kDriveEpsilon &&
             Math.abs(mStarboardPid.getError()) <= DriveConstants.kDriveEpsilon;
    }

    /**
     * Start Autonomous Rotation
     * @param theta the angle to rotate to
     */
    public void startAutoRotate(double theta) {
        mRotateSetpoint = theta; //TODO: Determine bounds on rotation
        mRotatePid.start(DriveConstants.kRotationalGains);
    }

    /**
     * Process Autonomous Rotation
     */
    public void runAutoRotate() {
        double theta = Util.normalizeAngle(mRotateSetpoint);
        double output = mRotatePid.updateRotation(getHeading(), theta);
        mDifferentialDrive.tankDrive(output, -output);
    }

    public boolean rotateTargetReached() {
        return Math.abs(mRotatePid.getError()) <= DriveConstants.kRotateEpsilon;
    }

    /**
     * Toggles between Max Robot Speed between Standard Speed and Slow Precision Speed
     */
    public void togglePrecisionDriving() {
        if(mSelectedMaxSpeedProportion == DriveConstants.kRegularMaxSpeed) {
            mSelectedMaxSpeedProportion = DriveConstants.kPrecisionMaxSpeed;
            mIsPrecisionMode = true;
        } else {
            mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
            mIsPrecisionMode = false;
        }
    }

    /**
     * Toggles Drivebase between Low and High Gearing
     * <p>If the Robot is in Precision Driving Mode, turns it off
     */
    public void toggleDrivebaseGearing(){
        switch (mGearShifter.get()) {
            case kForward: 
                mGearShifter.set(Value.kReverse);
                break;
            case kReverse: 
                mGearShifter.set(Value.kForward);
                break;
            default:
                mGearShifter.set(Value.kReverse);
                DriverStation.reportError("ERROR: GEAR SHIFT VALUE INVALID", false);
                break;
        }

        if(mIsPrecisionMode) {
            togglePrecisionDriving();
        }
    }

    public void tankDriveVolts(double portVolts, double starboardVolts) {
        mDriveGroupPort.setVoltage(portVolts);
        mDriveGroupStarboard.setVoltage(-starboardVolts);
        mDifferentialDrive.feed();
    }

    private double getPortEncoderRotations() {
        return mPortEncoder.getPosition() / DriveConstants.kDriveEncoderPPR;
    }

    private double getStarboardEncoderRotations() {
        return mStarboardEncoder.getPosition() / DriveConstants.kDriveEncoderPPR;
    }

    private double rotationsToInches(double rotations) {
        return rotations * DriveConstants.kWheelCircumference;
    }

    /**
     * @return Port Side Distance Travelled in Inches
     */
    private double getPortEncoderDistance() {
        return rotationsToInches(getPortEncoderRotations());
    }

    /**
     * @return Starboard Side Distance Travelled in Inches
     */
    private double getStarboardEncoderDistance() {
        return rotationsToInches(getStarboardEncoderRotations());
    }

    private double getEncoderCount(double distance) {
        return distance / DriveConstants.kWheelCircumference * DriveConstants.kDriveEncoderPPR; //TODO: have to factor in drive base gearing
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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getPortSpeed(), getStarboardSpeed());
    }

    /**
     * @return Port Side Linear Speed in Meters/second
     */
    private double getPortSpeed() {
        return getPortAngularVelocity() * DriveConstants.kWheelRadius;
    }

    /**
     * @return Starboard Side Linear Speed in Meters/second
     */
    private double getStarboardSpeed() {
        return getStarboardAngularVelocity() * DriveConstants.kWheelRadius;
    }

    private void resetEncoders() {
        mPortEncoder.setPosition(0.0);
        mStarboardEncoder.setPosition(0.0);
    }

    /**
     * Uses Right Hand Rule: CW is negative, CCW is positive
     * 
     * @return the robot's heading in degrees, from [-180, 180]
     */
    public double getHeading() {
        return -mNavx.getYaw();
    }

    /**
     * @return the robot's estimated pose
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * @return Differential Drive Kinematics Object
     */
    public DifferentialDriveKinematics getKinematics() {
        return mKinematics;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is High Gear", mGearShifter.get() == Value.kForward); //TODO: Find out if foward or reverse is high gear
        SmartDashboard.putBoolean("Precision Mode Enabled", mIsPrecisionMode);
        SmartDashboard.putNumber("Port Side Drive Speed", mDriveGroupPort.get());
        SmartDashboard.putNumber("Starboard Side Drive Speed", mDriveGroupStarboard.get());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    /**
     * @return DriveSubsystem Singleton Instance
     */
    public synchronized static DriveSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new DriveSubsystem();
        }
        return mInstance;
    }
}
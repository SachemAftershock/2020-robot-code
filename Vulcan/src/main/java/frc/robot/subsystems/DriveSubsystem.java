package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.PID;
import frc.robot.Util;

/**
 * Drivebase Subsystem for a Two Speed Transmission, 6 Wheel, West Coast Drive
 * 
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

    private double mPrevPow, mPrevRot;

    private double mPortSpeed, mStarboardSpeed, mLeftTarget, mRightTarget, mRotateSetpoint;

    private double mSelectedMaxSpeedProportion;
    private boolean mIsPrecisionMode;

    private boolean mIsAutoRotateRunning;
    
    /**
     * Constructor for DriveSubsystem Class
     */
    private DriveSubsystem() {
        mDriveMotorPortA = new CANSparkMax(DriveConstants.kDriveMotorPortAId, MotorType.kBrushless);
        mDriveMotorPortA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorPortB = new CANSparkMax(DriveConstants.kDriveMotorPortBId, MotorType.kBrushless);
        mDriveMotorPortB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorPortC = new CANSparkMax(DriveConstants.kDriveMotorPortCId, MotorType.kBrushless);
        mDriveMotorPortC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveGroupPort = new SpeedControllerGroup(mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC);
        addChild("Port Side Speed Controller Group",mDriveGroupPort);
                
        mDriveMotorStarboardA = new CANSparkMax(DriveConstants.kDriveMotorStarboardAId, MotorType.kBrushless);
        mDriveMotorStarboardA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
                
        mDriveMotorStarboardB = new CANSparkMax(DriveConstants.kDriveMotorStarboardBId, MotorType.kBrushless);
        mDriveMotorStarboardB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);

        mDriveMotorStarboardC = new CANSparkMax(DriveConstants.kDriveMotorStarboardCId, MotorType.kBrushless);
        mDriveMotorStarboardC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);

        mDriveGroupStarboard = new SpeedControllerGroup(mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC);
        addChild("Starboard Side Speed Controller Group", mDriveGroupStarboard);
                
        mDifferentialDrive = new DifferentialDrive(mDriveGroupPort, mDriveGroupStarboard);
        addChild("Differential Drive", mDifferentialDrive);
        mDifferentialDrive.setRightSideInverted(true);
        mDifferentialDrive.setSafetyEnabled(true);
        mDifferentialDrive.setExpiration(0.1);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);

        mPortEncoder = mDriveMotorPortA.getEncoder();
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();

        mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()), new Pose2d(0, 0, new Rotation2d()));
        //TODO: the auto I choose is gonna have to change the above x, y, and theta values
                
        mGearShifter = new DoubleSolenoid(Constants.kPcmAId, DriveConstants.kGearShiftForwardId, DriveConstants.kGearShiftReverseId);
        addChild("Gear Shift Double Solenoid", mGearShifter);    

        mPortPid = new PID();
        mStarboardPid = new PID();

        mRotatePid = new PID();

        mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
        mIsPrecisionMode = false;

        mIsAutoRotateRunning = false;

        mPortSpeed = 0.0;
        mStarboardSpeed = 0.0;
        mLeftTarget = 0.0;
        mRightTarget = 0.0;
        mRotateSetpoint = 0.0;
    }

    @Override
    public void init() {
        resetEncoders();
        mNavx.zeroYaw();
        mOdometry.resetPosition(new Pose2d(), new Rotation2d(getHeading()));
        mGearShifter.set(Value.kReverse); //TODO: Find out which side corresponds to which gearing, needs to start in Low Gear

        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
    }

    @Override
    public void periodic() {
        mOdometry.update(Rotation2d.fromDegrees(getHeading()), getPortEncoderDistance(), getStarboardEncoderDistance());
    }
    
    /**
     * Curvature Drive controlled via Xbox Controller
     * <p> 
     * Prevents sharp changes in acceleration
     * 
     * @param pow Robot's Linear Speed; Left Joystick Y-Axis
     * 
     * @param rot Robot's Rotation Rate; Right Joystick X-Axis
     * 
     * @param wantDeccelerate If the Operator requests the Robot to deccelerate; Left or Right Triggers
     */
    public void manualDrive(double pow, double rot, boolean wantDeccelerate) {
        if(wantDeccelerate) { 
            pow = mPrevPow * DriveConstants.kThrottleDecelerationProportion;
            rot = mPrevRot * DriveConstants.kRotationalDecelerationProportion; //TODO: Find out if we need to change rot differently than pow
        }

        pow *= mSelectedMaxSpeedProportion;
        rot *= mSelectedMaxSpeedProportion; //Same as above todo, not sure if we need to scale rot differently

        //If Velocity is changing by greater than the Max Acceptable Limit, the change in velocity is switched to the maximum acceptable change in velocity
        if(Math.abs(mPrevPow - pow) > DriveConstants.kMaxManualLinearAcceleration) {
            if(pow > 0) {
                if(pow > mPrevPow) {
                    pow = DriveConstants.kMaxManualLinearAcceleration + mPrevPow; //Forward & Accelerating
                } else {
                    pow = -DriveConstants.kMaxManualLinearAcceleration + mPrevPow; //Forward & Deccelerating
                }
            } else {
                if(pow < mPrevPow) {
                    pow = -DriveConstants.kMaxManualLinearAcceleration + mPrevPow; //Reverse & Accelerating
                } else {
                    pow = DriveConstants.kMaxManualLinearAcceleration + mPrevPow; //Reverse & Deccelerating
                }
            }
        }

        if(Math.abs(mPrevRot - rot) > DriveConstants.kMaxManualRotationAcceleration) {
            if(rot > 0) {
                if(rot > mPrevRot) {
                    rot = DriveConstants.kMaxManualRotationAcceleration + mPrevRot; //Positive & Accelerating
                } else {
                    rot = -DriveConstants.kMaxManualRotationAcceleration + mPrevRot; //Positive & Deccelerating
                }
            } else {
                if(rot < mPrevRot) {
                    rot = -DriveConstants.kMaxManualRotationAcceleration + mPrevRot; //Negative & Accelerating
                } else {
                    rot = DriveConstants.kMaxManualRotationAcceleration + mPrevRot; //Negative & Decelerating
                }
            }
        }

        mPrevPow = pow;
        mPrevRot = rot;

        mDifferentialDrive.curvatureDrive(pow, rot, true);
    }

    /**
     * Set target destination for Autonomous Linear Drive
     * 
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
     * <p>
     * Limited by Collision Avoidance if collision imminent 
     */
    public void runAutoDrive() {
        double scaleFactor = CollisionAvoidanceSubsystem.getInstance().getSlowdownScaleFactor();
        if(scaleFactor < 1.0) {
            mPortSpeed *= scaleFactor;
            mStarboardSpeed *= scaleFactor;
            mPortPid.pausePID();
            mStarboardPid.pausePID();
        } else {
            if(mPortPid.isPaused() || mStarboardPid.isPaused()) {
                mPortPid.resumePID();
                mStarboardPid.resumePID();
            }
            mPortSpeed = mPortPid.update(mPortEncoder.getPosition(), mLeftTarget);
            mStarboardSpeed = mStarboardPid.update(mStarboardEncoder.getPosition(), mRightTarget);
        }
        mDifferentialDrive.tankDrive(mPortSpeed, mStarboardSpeed);
        //If when driving straight, the robot is turning, add gyro-based correction
    }

    /**
     * Check for if the Linear Drive Target has been reached
     * 
     * @return <ul>
     *         <li> <i> true </i> when Port Error & Starboard Error < Drive Epsilon
     *         <li> <i> false </i> otherwise
     *         </ul>
     */
    public boolean linearDriveTargetReached() {
        return Math.abs(mPortPid.getError()) <= DriveConstants.kDriveEpsilon &&
             Math.abs(mStarboardPid.getError()) <= DriveConstants.kDriveEpsilon;
    }

    /**
     * Start Autonomous Rotation
     * 
     * @param theta the angle to rotate to
     */
    public void startAutoRotate(double theta) {
        mIsAutoRotateRunning = true;
        mRotateSetpoint = Util.normalizeAngle(theta);
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

    /**
     * Check for if the Rotational Drive Target has been reached
     * 
     * @return <i>true</i> when Rotational Error < Epsilon; <i>false</i> otherwise
     */
    public boolean rotateTargetReached() {
        final boolean targetReached = Math.abs(mRotatePid.getError()) <= DriveConstants.kRotateEpsilon;
        if(targetReached) {
            mIsAutoRotateRunning = false;
        }
        return targetReached;
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
     * <p>
     * If the Robot is in Precision Driving Mode, turns it off
     */
    public void toggleDrivebaseGearing(){
        switch (mGearShifter.get()) {
            case kForward: 
                mGearShifter.set(Value.kReverse);
                mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
                mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
                break;
            case kReverse: 
                mGearShifter.set(Value.kForward);
                mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kHighGearRatio);
                mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kHighGearRatio);
                break;
            default:
                mGearShifter.set(Value.kReverse);
                mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
                mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
                DriverStation.reportError("ERROR: GEAR SHIFT VALUE INVALID", false);
                break;
        }

        if(mIsPrecisionMode) {
            togglePrecisionDriving();
        }
    }

    /**
     * Checks if Drivebase Gearing is in High Gear
     * 
     * @return <i>true</i> when Gear Solenoid is Forward, i.e in High Gear; <i>false</i> otherwise
     */
    public boolean isHighGear() {
        return mGearShifter.get() == Value.kForward; //TODO: Find out if this is actually high
    }

    /**
     * Checks if Precision Mode is Enabled
     * 
     * @return <i>true</i> when Precision Mode Enabled <i>false</i> otherwise
     */
    public boolean isPrecisionMode() {
        return mIsPrecisionMode;
    }

    /**
     * Used for Ramsete Controller
     * <p>
     * Independently drives Port and Starboard side using Voltage
     * 
     * @param portVolts Voltage used to drive the Port Side
     * 
     * @param starboardVolts Voltage used to drive the Starboard Side
     */
    public void tankDriveVolts(double portVolts, double starboardVolts) {
        mDriveGroupPort.setVoltage(portVolts);
        mDriveGroupStarboard.setVoltage(-starboardVolts);
        mDifferentialDrive.feed();
    }

    /**
     * Gets Number of Wheel Rotations on the Port Side
     * 
     * @return Number of Port Side Wheel Rotations
     */
    private double getPortEncoderRotations() {
        return mPortEncoder.getPosition() / DriveConstants.kDriveEncoderPPR;
    }

    /**
     * Gets Number of Wheel Rotations on the Starboard Side
     * 
     * @return Number of Starboard Side Wheel Rotations
     */
    private double getStarboardEncoderRotations() {
        return mStarboardEncoder.getPosition() / DriveConstants.kDriveEncoderPPR;
    }

    /**
     * Converts Wheel Rotations to Linear Distance travelled
     * 
     * @param rotations number of rotations
     * @return linear distance travelled
     */
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

    /**
     * Gets Number of Encoder Pulses for a distance to travel linearly
     * 
     * @param distance the distance to travel linearly
     * 
     * @return the number of encoder pulses to travel the distance
     */
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

    /**
     * Zeros Port & Starboard Drive Encoders
     */
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

    /**
     * Gets if a Autonomous Rotation Command is running
     * 
     * @return <i> true </i> if the Robot is autonomously rotating; <i> false </i> otherwise
     */
    public boolean getIsAutoRotateRunning() {
        return mIsAutoRotateRunning;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putBoolean("Is High Gear", mGearShifter.get() == Value.kForward); //TODO: Find out if foward or reverse is high gear
        SmartDashboard.putBoolean("Precision Mode Enabled", mIsPrecisionMode);
        SmartDashboard.putNumber("Port Side Drive Speed", mDriveGroupPort.get());
        SmartDashboard.putNumber("Starboard Side Drive Speed", mDriveGroupStarboard.get());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    @Override
    public void runTest() {
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
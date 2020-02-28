package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
 * <p>
 * {@link CollisionAvoidanceSubsystem Features Proportional Collision Avoidance}
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
    private boolean mInPrecisionMode;

    private boolean mAutoRotateRunning;
    
    /**
     * Constructor for DriveSubsystem Class
     */
    private DriveSubsystem() {
        //I'm only this thorough with Spark initialization because I saw a thread on Chief Delphi 
        //that Spark MAXs reset to the settings burned into the flash memory if they lose power
        mDriveMotorPortA = new CANSparkMax(DriveConstants.kDriveMotorPortAId, MotorType.kBrushless);
        mDriveMotorPortA.restoreFactoryDefaults();
        mDriveMotorPortA.setMotorType(MotorType.kBrushless);
        mDriveMotorPortA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorPortA.setIdleMode(IdleMode.kBrake);
        mDriveMotorPortA.setInverted(false);
        mDriveMotorPortA.burnFlash();
                
        mDriveMotorPortB = new CANSparkMax(DriveConstants.kDriveMotorPortBId, MotorType.kBrushless);
        mDriveMotorPortB.restoreFactoryDefaults();
        mDriveMotorPortB.setMotorType(MotorType.kBrushless);
        mDriveMotorPortB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorPortB.setIdleMode(IdleMode.kBrake);
        mDriveMotorPortB.setInverted(false);
        mDriveMotorPortB.burnFlash();

                
        mDriveMotorPortC = new CANSparkMax(DriveConstants.kDriveMotorPortCId, MotorType.kBrushless);
        mDriveMotorPortC.restoreFactoryDefaults();
        mDriveMotorPortC.setMotorType(MotorType.kBrushless);
        mDriveMotorPortC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorPortC.setIdleMode(IdleMode.kBrake);
        mDriveMotorPortC.setInverted(false);
        mDriveMotorPortC.burnFlash();
                
        mDriveGroupPort = new SpeedControllerGroup(mDriveMotorPortA, mDriveMotorPortB, mDriveMotorPortC);
        addChild("Port Side Speed Controller Group",mDriveGroupPort);
                
        mDriveMotorStarboardA = new CANSparkMax(DriveConstants.kDriveMotorStarboardAId, MotorType.kBrushless);
        mDriveMotorStarboardA.restoreFactoryDefaults();
        mDriveMotorStarboardA.setMotorType(MotorType.kBrushless);
        mDriveMotorStarboardA.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorStarboardA.setIdleMode(IdleMode.kBrake);
        mDriveMotorStarboardA.setInverted(false);
        mDriveMotorStarboardA.burnFlash();
                
        mDriveMotorStarboardB = new CANSparkMax(DriveConstants.kDriveMotorStarboardBId, MotorType.kBrushless);
        mDriveMotorStarboardB.restoreFactoryDefaults();
        mDriveMotorStarboardB.setMotorType(MotorType.kBrushless);
        mDriveMotorStarboardB.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorStarboardB.setIdleMode(IdleMode.kBrake);
        mDriveMotorStarboardB.setInverted(false);
        mDriveMotorStarboardB.burnFlash();

        mDriveMotorStarboardC = new CANSparkMax(DriveConstants.kDriveMotorStarboardCId, MotorType.kBrushless);
        mDriveMotorStarboardC.restoreFactoryDefaults();
        mDriveMotorStarboardC.setMotorType(MotorType.kBrushless);
        mDriveMotorStarboardC.setOpenLoopRampRate(DriveConstants.kRampRateToMaxSpeed);
        mDriveMotorStarboardC.setIdleMode(IdleMode.kBrake);
        mDriveMotorStarboardC.setInverted(false);
        mDriveMotorStarboardC.burnFlash();

        mDriveGroupStarboard = new SpeedControllerGroup(mDriveMotorStarboardA, mDriveMotorStarboardB, mDriveMotorStarboardC);
        addChild("Starboard Side Speed Controller Group", mDriveGroupStarboard);
                
        mDifferentialDrive = new DifferentialDrive(mDriveGroupPort, mDriveGroupStarboard);
        addChild("Differential Drive", mDifferentialDrive);
        mDifferentialDrive.setRightSideInverted(false);
        mDifferentialDrive.setSafetyEnabled(true);
        mDifferentialDrive.setExpiration(0.1);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);

        mPortEncoder = mDriveMotorPortA.getEncoder();
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();

        mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()), new Pose2d(0, 0, new Rotation2d()));
        //TODO: the auto I choose is gonna have to change the above x, y, and theta values
                
        mGearShifter = new DoubleSolenoid(Constants.kPcmId, DriveConstants.kGearShiftForwardId, DriveConstants.kGearShiftReverseId);
        addChild("Gear Shift Double Solenoid", mGearShifter);    

        mPortPid = new PID();
        mStarboardPid = new PID();

        mRotatePid = new PID();

        mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
        mInPrecisionMode = false;

        mAutoRotateRunning = false;

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
        mOdometry.update(Rotation2d.fromDegrees(getHeading()), getPortEncoderDistanceInches(), getStarboardEncoderDistanceInches());
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
     * 
     * @see frc.robot.commands.drive.ManualDriveCommand
     */
    public void manualDrive(double pow, double rot, boolean wantDeccelerate) {
        if(wantDeccelerate) { 
            pow = mPrevPow * DriveConstants.kThrottleDecelerationProportion;
            rot = mPrevRot * DriveConstants.kRotationalDecelerationProportion; //TODO: Find out if we need to change rot differently than pow
        }

        pow *= mSelectedMaxSpeedProportion;
        rot *= mSelectedMaxSpeedProportion; //Same as above todo, not sure if we need to scale rot differently

        final double slowdownScaleFactor = CollisionAvoidanceSubsystem.getInstance().getSlowdownScaleFactor();
        pow *= slowdownScaleFactor;
        rot *= slowdownScaleFactor;

        //Acceleration Limiting
        if(Math.abs(mPrevPow - pow) > DriveConstants.kMaxManualLinearAcceleration) {
            if(pow - mPrevPow > 0) {
                pow = mPrevPow + DriveConstants.kMaxManualLinearAcceleration; //Accelerating in positive direction
            } else {
                pow = mPrevPow - DriveConstants.kMaxManualLinearAcceleration; //Accelerating in negative direction
            }
        }

        if(Math.abs(mPrevRot - rot) > DriveConstants.kMaxManualRotationAcceleration) {
            if(rot - mPrevRot > 0) {
                rot = mPrevRot + DriveConstants.kMaxManualRotationAcceleration;
            } else {
                rot = mPrevRot - DriveConstants.kMaxManualRotationAcceleration;
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
     * 
     * @see frc.robot.commands.drive.LinearDriveCommand
     */
    public void startAutoDrive(double setpoint) {
        mLeftTarget = mPortEncoder.getPosition() + getRotations(setpoint);
        mRightTarget = mStarboardEncoder.getPosition() + getRotations(setpoint);
        mPortPid.start(DriveConstants.kLinearGains);
        mStarboardPid.start(DriveConstants.kLinearGains);
    }

    /**
     * Process Autonomous Linear Drive
     * <p>
     * Limited by Collision Avoidance if collision imminent 
     * 
     * @see frc.robot.commands.drive.LinearDriveCommand
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
            mPortSpeed = mPortPid.update(getPortRotations(), mLeftTarget);
            mStarboardSpeed = mStarboardPid.update(getStarboardRotations(), mRightTarget);
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
     * 
     * @see frc.robot.commands.drive.LinearDriveCommand
     */
    public boolean linearDriveTargetReached() {
        return Math.abs(mPortPid.getError()) <= DriveConstants.kDriveEpsilon &&
             Math.abs(mStarboardPid.getError()) <= DriveConstants.kDriveEpsilon;
    }

    /**
     * Start Autonomous Rotation
     * 
     * @param theta the angle to rotate to
     * 
     * @see frc.robot.commands.drive.RotateDriveCommand
     */
    public void startAutoRotate(double theta) {
        mAutoRotateRunning = true;
        mRotateSetpoint = Util.normalizeAngle(theta);
        mRotatePid.start(DriveConstants.kRotationalGains);
    }

    /**
     * Process Autonomous Rotation
     * 
     * @see frc.robot.commands.drive.RotateDriveCommand
     */
    public void runAutoRotate() {
        double theta = Util.normalizeAngle(mRotateSetpoint);
        double output = mRotatePid.updateRotation(getHeading(), theta);
        mDifferentialDrive.tankDrive(output, -output);
    }

    /**
     * Check for if the Rotational Drive Target has been reached
     * 
     * @return <i> true </i> when Rotational Error < Epsilon; <i> false </i> otherwise
     * 
     * @see frc.robot.commands.drive.RotateDriveCommand
     */
    public boolean rotateTargetReached() {
        final boolean targetReached = Math.abs(mRotatePid.getError()) <= DriveConstants.kRotateEpsilon;
        if(targetReached) {
            mAutoRotateRunning = false;
        }
        return targetReached;
    }

    /**
     * Toggles between Max Robot Speed between Standard Speed and Slow Precision Speed
     * 
     * @see frc.robot.commands.drive.TogglePrecisionDrivingCommand
     */
    public void togglePrecisionDriving() {
        if(mSelectedMaxSpeedProportion == DriveConstants.kRegularMaxSpeed) {
            mSelectedMaxSpeedProportion = DriveConstants.kPrecisionMaxSpeed;
            mInPrecisionMode = true;
        } else {
            mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
            mInPrecisionMode = false;
        }
    }

    /**
     * Toggles Drivebase between Low and High Gearing
     * <p>
     * If the Robot is in Precision Driving Mode, turns it off
     * 
     * @see frc.robot.commands.drive.ToggleDrivebaseGearingCommand
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

        if(mInPrecisionMode) {
            togglePrecisionDriving();
        }
    }

    /**
     * Checks if Drivebase Gearing is in High Gear
     * 
     * @return <i> true </i> when Gear Solenoid is Forward, i.e in High Gear; <i> false </i> otherwise
     */
    public synchronized boolean isHighGear() {
        return mGearShifter.get() == Value.kForward; //TODO: Find out if this is actually high
    }

    /**
     * Checks if Precision Mode is Enabled
     * 
     * @return <i> true </i> when Precision Mode Enabled <i> false </i> otherwise
     */
    public boolean isPrecisionMode() {
        return mInPrecisionMode;
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
    private double getPortRotations() {
        return mPortEncoder.getPosition();
    }

    /**
     * Gets Number of Wheel Rotations on the Starboard Side
     * 
     * @return Number of Starboard Side Wheel Rotations
     */
    private double getStarboardRotations() {
        return mStarboardEncoder.getPosition();
    }

    /**
     * Converts Wheel Rotations to Linear Distance travelled
     * 
     * @param rotations number of rotations
     * 
     * @return linear distance travelled
     */
    private double rotationsToInches(double rotations) {
        return rotations * DriveConstants.kWheelCircumferenceInches;
    }

    /**
     * @return Port Side Distance Travelled in Inches
     */
    private double getPortEncoderDistanceInches() {
        return rotationsToInches(getPortRotations());
    }

    /**
     * @return Starboard Side Distance Travelled in Inches
     */
    private double getStarboardEncoderDistanceInches() {
        return rotationsToInches(getStarboardRotations());
    }

    /**
     * Gets Number of Rotations for a distance to travel linearly
     * 
     * @param distanceInches the distance to travel linearly in inches
     * 
     * @return the number of rotations to travel the distance
     */
    private double getRotations(double distanceInches) {
        if(isHighGear()) {
            return distanceInches / DriveConstants.kWheelCircumferenceInches * DriveConstants.kHighGearRatio;
        }
        return distanceInches / DriveConstants.kWheelCircumferenceInches  * DriveConstants.kLowGearRatio;
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

    /**
     * Gets Current Wheel Speeds
     * 
     * @return DifferentialDriveWheelSpeeds object containing Port & Starboard side speeds
     */
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
    public boolean isAutoRotateRunning() {
        return mAutoRotateRunning;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putBoolean("Is High Gear", mGearShifter.get() == Value.kForward); //TODO: Find out if foward or reverse is high gear
        SmartDashboard.putBoolean("Precision Mode Enabled", mInPrecisionMode);
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
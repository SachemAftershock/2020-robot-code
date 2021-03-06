package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PneumaticConstants;
import frc.lib.AftershockSubsystem;
import frc.lib.PID;
import frc.lib.Util;

/**
 * Drivebase Subsystem for a Two Speed Transmission, 6 Wheel, West Coast Drive
 * <p>
 * {@link CollisionAvoidanceSubsystem Features Proportional Collision Avoidance}
 * 
 * @author Shreyas Prasad
 */
public class DriveSubsystem extends AftershockSubsystem {

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

    private boolean mManualDriveInverted;
    
    /**
     * Constructor for DriveSubsystem Class
     */
    private DriveSubsystem() {
        super();
        setName("Drive Subsystem");
        
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
        mDifferentialDrive.setRightSideInverted(true);
        mDifferentialDrive.setSafetyEnabled(false);
        mDifferentialDrive.setMaxOutput(1.0);

        mNavx = new AHRS(Port.kMXP);
        addChild("Gyro", mNavx);

        mPortEncoder = mDriveMotorPortA.getEncoder();
        mStarboardEncoder = mDriveMotorStarboardA.getEncoder();

        mKinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidthMeters);

        mOdometry = new DifferentialDriveOdometry(new Rotation2d(getHeadingRadians()), new Pose2d(0, 0, new Rotation2d()));
        //TODO: Need some way to make the start position field-centric instead of robot-centric
        //maybe lidars pointed against 2 walls?
                
        mGearShifter = new DoubleSolenoid(PneumaticConstants.kPcmId, DriveConstants.kGearShiftForwardId, DriveConstants.kGearShiftReverseId);
        addChild("Gear Shift Double Solenoid", mGearShifter);    

        mPortPid = new PID();
        mStarboardPid = new PID();

        mRotatePid = new PID();

        mSelectedMaxSpeedProportion = DriveConstants.kRegularMaxSpeed;
        mInPrecisionMode = false;

        mAutoRotateRunning = false;

        mManualDriveInverted = false;

        mPortSpeed = 0.0;
        mStarboardSpeed = 0.0;
        mLeftTarget = 0.0;
        mRightTarget = 0.0;
        mRotateSetpoint = 0.0;
    }

    @Override
    public void initialize() {
        mDifferentialDrive.tankDrive(0.0, 0.0);
        mManualDriveInverted = false;
        
        resetEncoders();
        mNavx.zeroYaw();
        //Need a way to find robot position on field, not just assume we start at 0,0
        //See above todo next to mOdometry initializtion
        mOdometry.resetPosition(new Pose2d(), new Rotation2d(getHeading()));
        shiftLowGear();

        //If you're from the future looking at this code in reference, get an encoder at the output of the transmission
        //Right now, I'm using math to account for the gears in the drive transmission
        //This probably won't work accurately, especially for more complex autonomous paths
        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
    }

    @Override
    public void periodic() {
        mOdometry.update(Rotation2d.fromDegrees(getHeading()), getPortEncoderDistanceMeters(), getStarboardEncoderDistanceMeters());
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
     * @param wantDeccelerate If the Driver requests the Robot to deccelerate; Left or Right Triggers
     * 
     * @see frc.robot.commands.drive.ManualDriveCommand
     */
    public void manualDrive(double pow, double rot, boolean wantDeccelerate) {
        rot = -rot;
        rot *= DriveConstants.kManualDriveRotationScaleFactor;

        if(mManualDriveInverted) {
            pow = -pow;
        }

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

        //This actually shouldn't be a constant true for isQuickTurn, that somewhat invalidates
        //the point of running curvature drive; try running without quick turn in the future
        //(or with a button to control it)
        mDifferentialDrive.curvatureDrive(-pow, rot, true);
    }

    /**
     * Set target destination for Autonomous Linear Drive
     * 
     * @param setpoint target destination to drive forward to in inches
     * 
     * @see frc.robot.commands.drive.LinearDriveCommand
     */
    public void startAutoDrive(double setpoint) {
        mLeftTarget = getPortRotations() + inchesToRotations(setpoint);
        mRightTarget = getStarboardRotations() + inchesToRotations(setpoint);
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
    public boolean isLinearDriveTargetReached() {
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

    public void stop() {
        mDifferentialDrive.tankDrive(0, 0);
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

    public void shiftHighGear() {
        mGearShifter.set(Value.kForward);
        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kHighGearRatio);
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kHighGearRatio);
        if(mInPrecisionMode) {
            togglePrecisionDriving();
        }
    }

    public void shiftLowGear() {
        mGearShifter.set(Value.kReverse);
        mPortEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
        mStarboardEncoder.setVelocityConversionFactor((1/ 60) * 2 * Math.PI * DriveConstants.kLowGearRatio);
        if(mInPrecisionMode) {
            togglePrecisionDriving();
        }
    }

    public void invertManualDrive() {
        mManualDriveInverted = true;
    }

    public void revertManualDrive() {
        mManualDriveInverted = false;
    }

    public boolean isManualDriveInverted() {
        return mManualDriveInverted;
    }

    /**
     * Checks if Drivebase Gearing is in High Gear
     * 
     * @return <i> true </i> when Gear Solenoid is Forward, i.e in High Gear; <i> false </i> otherwise
     */
    public synchronized boolean isHighGear() {
        return mGearShifter.get() == Value.kForward;
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
     * <p>
     * Negated as Starboard Side is inverted
     * 
     * @return Number of Starboard Side Wheel Rotations
     */
    private double getStarboardRotations() {
        return -mStarboardEncoder.getPosition();
    }

    /**
     * Converts Wheel Rotations to Linear Distance travelled in Inches
     * 
     * @param rotations number of rotations
     * 
     * @return linear distance travelled in inches
     */
    private double rotationsToInches(double rotations) {
        if(isHighGear()) {
            return rotations / DriveConstants.kHighGearRatio * DriveConstants.kWheelCircumference;
        }
        return rotations / DriveConstants.kLowGearRatio * DriveConstants.kWheelCircumference;
    }

    /**
     * @return Port Side Distance Travelled in Inches
     */
    public double getPortEncoderDistance() {
        return rotationsToInches(getPortRotations());
    }

    /**
     * @return Starboard Side Distance Travelled in Inches
     */
    public double getStarboardEncoderDistance() {
        return rotationsToInches(getStarboardRotations());
    }

    /**
     * Converts Wheel Rotations to Linear Distance travelled in Meters
     * 
     * @param rotations number of rotations
     * 
     * @return linear distance travelled in meters
     */
    private double rotationsToMeters(double rotations) {
        if(isHighGear()) {
            return (rotations / DriveConstants.kHighGearRatio) * DriveConstants.kWheelCircumferenceMeters;
        }
        return (rotations / DriveConstants.kLowGearRatio) * DriveConstants.kWheelCircumferenceMeters;
    }

    /**
     * @return Port Side Distance Travelled in Meters
     */
    private double getPortEncoderDistanceMeters() {
        return rotationsToMeters(getPortRotations());
    }

    /**
     * @return Starboard Side Distance Travelled in Meters
     */
    private double getStarboardEncoderDistanceMeters() {
        return rotationsToMeters(getStarboardRotations());
    }

    /**
     * Gets Number of Rotations for a distance to travel linearly
     * 
     * @param distanceInches the distance to travel linearly in inches
     * 
     * @return the number of rotations to travel the distance
     */
    private double inchesToRotations(double distanceInches) {
        if(isHighGear()) {
            return (distanceInches / DriveConstants.kWheelCircumference) * DriveConstants.kHighGearRatio;
        }
        return (distanceInches / DriveConstants.kWheelCircumference)  * DriveConstants.kLowGearRatio;
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
     * Gets Current Wheel Speeds in Feet/second
     * 
     * @return DifferentialDriveWheelSpeeds object containing Port & Starboard side speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getPortSpeed(), getStarboardSpeed());
    }

    /**
     * @return Port Side Linear Speed in Feet/second
     */
    private double getPortSpeed() {
        return getPortAngularVelocity() * DriveConstants.kWheelRadius;
    }

    /**
     * @return Starboard Side Linear Speed in Feet/second
     */
    private double getStarboardSpeed() {
        return getStarboardAngularVelocity() * DriveConstants.kWheelRadius;
    }

    /**
     * Gets Current Wheel Speeds in Meters/second
     * 
     * @return DifferentialDriveWheelSpeeds object containing Port & Starboard side speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeedsMetersPerSecond() {
        return new DifferentialDriveWheelSpeeds(getPortSpeedMetersPerSecond(), getStarboardSpeedMetersPerSecond());
    }

    /**
     * @return Port Side Linear Speed in Meters/second
     */
    private double getPortSpeedMetersPerSecond() {
        return getPortAngularVelocity() * DriveConstants.kWheelRadiusMeters;
    }

    /**
     * @return Starboard Side Linear Speed in Meters/second
     */
    private double getStarboardSpeedMetersPerSecond() {
        return getStarboardAngularVelocity() * DriveConstants.kWheelRadiusMeters;
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
        final double heading = mNavx.getYaw();
        if(heading == 0.0) {
            return 0.0; //prevents a -0.0 reading
        }
        return -heading;
    }

    /**
     * Uses Right Hand Rule: CW is negative, CCW is positive
     * 
     * @return the robot's heading in radians, from [-pi, pi]
     */
    public double getHeadingRadians() {
        return Math.toRadians(getHeading());
    }

    /**
     * @return the robot's estimated pose
     */
    public synchronized Pose2d getPoseMeters() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Returns Estimated Pose of the Robot with units in inches
     * 
     * @return estimated robot Pose2d in inches
     */
    public synchronized Pose2d getPoseInches() {
        final Pose2d estimatedPoseMeters = getPoseMeters();
        final Translation2d estimatedTranslation2dMeters = estimatedPoseMeters.getTranslation();
        final double x = Units.metersToInches(estimatedTranslation2dMeters.getX());
        final double y = Units.metersToInches(estimatedTranslation2dMeters.getY());
        return new Pose2d(x, y, estimatedPoseMeters.getRotation());
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
        SmartDashboard.putBoolean("Is High Gear", isHighGear()); 
        SmartDashboard.putBoolean("Precision Mode Enabled", mInPrecisionMode);
        SmartDashboard.putNumber("Heading", getHeading());
        //SmartDashboard.putNumber("Port Speed", getPortSpeed());
        //SmartDashboard.putNumber("Starboard Speed", getStarboardSpeed());
        SmartDashboard.putNumber("Port Rotations", getPortRotations());
        SmartDashboard.putNumber("Starboard Rotations", getStarboardRotations());
        SmartDashboard.putNumber("Port Target", mLeftTarget);
        SmartDashboard.putNumber("Starboard Target", mRightTarget);
        SmartDashboard.putBoolean("Linear Drive Target Reached", isLinearDriveTargetReached());
    }

    @Override
    public boolean checkSystem() {
        return true;
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
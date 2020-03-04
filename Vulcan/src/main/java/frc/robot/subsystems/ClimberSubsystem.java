package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


/**
 * Climber Subsystem
 * 
 * @author Shreyas Prasad
 */
public class ClimberSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ClimberSubsystem mInstance;

    private final WPI_TalonSRX mElevator, mLifter;

    private ClimbElevatorPosition mCurrentPosition, mDesiredPosition;

    private boolean mHasLifterRun;
    
    /**
     * Constructor for ClimberSubsystem Class
     */
    private ClimberSubsystem() {
        mElevator = new WPI_TalonSRX(ClimberConstants.kElevatorId);
        mElevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, ClimberConstants.kElevatorPidId, 0);
        mElevator.setInverted(InvertType.None);
        mElevator.setSensorPhase(false);
        mElevator.config_kP(ClimberConstants.kElevatorPidId, ClimberConstants.kGains[0]);
        mElevator.config_kI(ClimberConstants.kElevatorPidId, ClimberConstants.kGains[1]);
        mElevator.config_kD(ClimberConstants.kElevatorPidId, ClimberConstants.kGains[2]);
        mElevator.configMotionAcceleration(ClimberConstants.kMagicAcceleration, 0);
        mElevator.configMotionCruiseVelocity(ClimberConstants.kMagicCruiseVelocity, 0);
		mElevator.config_IntegralZone(ClimberConstants.kElevatorPidId, ClimberConstants.kIntegralZone, 0);
		mElevator.configClosedloopRamp(0, ClimberConstants.kTimeout);
        mElevator.configAllowableClosedloopError(ClimberConstants.kElevatorPidId, ClimberConstants.kEpsilon, 0);
        addChild("Climb Elevator", mElevator);

        mLifter = new WPI_TalonSRX(ClimberConstants.kLifterId);
        addChild("Winch", mLifter);

        mHasLifterRun = false;
    }

    @Override
    public void init() {
        mHasLifterRun = false;
        mElevator.setSelectedSensorPosition(0); //Stowed Position

        mCurrentPosition = ClimbElevatorPosition.eLow;
        mDesiredPosition = ClimbElevatorPosition.eLow;
    }

    /**
     * Raises Elevator Level by one level
     * <p>
     * Does nothing if at highest point (eHigh)
     * 
     * @see frc.robot.commands.climber.RaiseElevatorLevelCommand
     */
    public void raiseElevatorLevel() {
        switch(mCurrentPosition) {
            case eStart:
                mDesiredPosition = ClimbElevatorPosition.eMid;
                mElevator.set(ControlMode.MotionMagic, mCurrentPosition.getEncoderCount());
                break;
            case eLow:
                break;
            case eMid:
                mDesiredPosition = ClimbElevatorPosition.eHigh;
                mElevator.set(ControlMode.MotionMagic, mCurrentPosition.getEncoderCount());
                break;
            case eHigh:
            default:
        }
    }

    /**
     * Lowers Elevator by one level
     * <p>
     * Does nothing if at lowest point (eLow)
     * 
     * @see frc.robot.commands.climber.LowerElevatorLevelCommand
     */
    public void lowerElevatorLevel() {
        switch(mCurrentPosition) {
            case eHigh:
                mDesiredPosition = ClimbElevatorPosition.eMid;
                mElevator.set(ControlMode.MotionMagic, mCurrentPosition.getEncoderCount());
                break;
            case eMid:
                mDesiredPosition = ClimbElevatorPosition.eLow;
                mElevator.set(ControlMode.MotionMagic, mCurrentPosition.getEncoderCount());
                break;
            case eLow:
            default:
        }
    }

    public void forwardElevator() {
        mElevator.set(ControlMode.PercentOutput, 1.0);
    }

    public void reverseElevator() {
        mElevator.set(ControlMode.PercentOutput, -0.75);
    }

    public void stopElevator() {
        mElevator.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Checks if the elevator has travelled from the last elevator position to the desired elevator position
     * 
     * @return <i> true </i> if elevator has reached desired position; <i> false </i> otherwise
     * 
     * @see frc.robot.commands.climber.RaiseElevatorLevelCommand
     * 
     * @see frc.robot.commands.climber.LowerElevatorLevelCommand
     */
    public boolean isAtDesiredPosition() {
        final boolean atDesiredPosition = Math.abs(mElevator.getSelectedSensorPosition() - mDesiredPosition.getEncoderCount()) <= ClimberConstants.kEpsilon;
        if(atDesiredPosition) {
            mCurrentPosition = mDesiredPosition;
        }
        return atDesiredPosition;
    }

    /**
     * Runs Lift Winch at constant speed
     */
    public void driveLifter() {
        mLifter.set(ControlMode.PercentOutput, ClimberConstants.kLifterSpeed);
        mHasLifterRun = true;
    }

    /**
     * Stops Lift Winch
     */
    public void stopLifter() {
        mLifter.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Elevator Positions containing the encoder count for the appropriate position
     * 
     * @author Shreyas Prasad
     */
    public enum ClimbElevatorPosition {
        eStart(ClimberConstants.kElevataorLevelEncoderValues[0]),
        eLow(ClimberConstants.kElevataorLevelEncoderValues[1]),
        eMid(ClimberConstants.kElevataorLevelEncoderValues[2]),
        eHigh(ClimberConstants.kElevataorLevelEncoderValues[3]); 

        private final double mEncoderCount;

        private ClimbElevatorPosition(double encoderCount) {
            mEncoderCount = encoderCount;
        }

        /**
         * Get required encoder count for the elevator level
         * @return CTRE Mag Encoder Counts for the elevator level
         */
        public double getEncoderCount() {
            return mEncoderCount;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }

    /**
     * Gets if Lift Winch Motor is running
     * <p>
     * Used for {@link LEDSubsystem#periodic() LED Feedback}
     * 
     * @return <i> true </i> if Lift Winch Motor is running; <i> false </i> otherwise
     * 
     */
    public boolean isClimbing() {
        return Math.abs(mLifter.get()) > 0;
    }

    /**
     * Predicts if the Robot is finished climbing based off if the Lift Winch Motor has previously ran and has now stopped running
     * <p>
     * Used for {@link LEDSubsystem#periodic() LED Feedback}
     * 
     * @return <i> true </i> if the Robot is predicted to have finished climbing; <i> false </i> otherwise
     */
    public boolean predictIsClimbFinished() {
        return mHasLifterRun && Math.abs(mLifter.get()) == 0.0;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Current Encoder Count", mElevator.getSelectedSensorPosition());
        SmartDashboard.putString("Current Position", mCurrentPosition.toString());
        SmartDashboard.putString("Desired Position", mDesiredPosition.toString());
    }

    @Override
    public void runTest() {
    }

    /**
     * @return ClimberSubsystem Singleton Instance
     */
    public synchronized static ClimberSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ClimberSubsystem();
        }
        return mInstance;
    }
}


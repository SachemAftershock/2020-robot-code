package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.commands.climber.LowerElevatorLevelCommand;
import frc.robot.commands.climber.RaiseElevatorLevelCommand;
import frc.robot.commands.climber.RunClimber;

/**
 * Climber Subsystem
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

        mLifter = new WPI_TalonSRX(ClimberConstants.kLifterId);
        
        mCurrentPosition = ClimbElevatorPosition.eLow;
        mDesiredPosition = ClimbElevatorPosition.eLow;

        mHasLifterRun = false;
    }

    @Override
    public void init() {
        mHasLifterRun = false;
        mElevator.setSelectedSensorPosition(0); //Stowed Position
    }

    /**
     * Raises Elevator Level by one level
     * <p>
     * Does nothing if at highest point
     */
    public void raiseElevatorLevel() {
        switch(mCurrentPosition) {
            case eLow:
                mDesiredPosition = ClimbElevatorPosition.eMid;
                mElevator.set(ControlMode.MotionMagic, mCurrentPosition.getEncoderCount());
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
     * Does nothing if at lowest point
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

    /**
     * Checks if the elevator has travelled from the last elevator position to the desired elevator position
     * @return <i> true </i> if elevator has reached desired position; <i> false </i> otherwise
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
     * @author Shreyas Prasad
     */
    public enum ClimbElevatorPosition implements Sendable {
        eLow(ClimberConstants.kElevataorLevelEncoderValues[0]),
        eMid(ClimberConstants.kElevataorLevelEncoderValues[1]),
        eHigh(ClimberConstants.kElevataorLevelEncoderValues[2]); 

        private final int mEncoderCount;

        private ClimbElevatorPosition(int encoderCount) {
            mEncoderCount = encoderCount;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            //Not sure anything is needed here
        }

        /**
         * Get required encoder count for the elevator level
         * @return CTRE Mag Encoder Counts for the elevator level
         */
        public int getEncoderCount() {
            return mEncoderCount;
        }
    }

    /**
     * Gets if Lift Winch Motor is running
     * <p>
     * Used for LED Feedback
     * @return <i> true </i> if Lift Winch Motor is running; <i> false </i> otherwise
     */
    public boolean isClimbing() {
        return Math.abs(mLifter.get()) > 0;
    }

    /**
     * Predicts if the Robot is finished climbing based off if the Lift Winch Motor has previously ran and has now stopped running
     * @return <i> true </i> if the Robot is predicted to have finished climbing; <i> false </i> otherwise
     */
    public boolean predictIsClimbFinished() {
        return mHasLifterRun && Math.abs(mLifter.get()) == 0.0;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putNumber("Current Encoder Count", mElevator.getSelectedSensorPosition());
        SmartDashboard.putData("Current Position", mCurrentPosition);
        SmartDashboard.putData("Desired Position", mDesiredPosition);
    }

    @Override
    public void runTest() {
        final ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        NetworkTableEntry elevatorSpeed = tab.add("Elevator Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1.0, "max", 1)).getEntry();
        SmartDashboard.putData("Run Elevator", new InstantCommand(() -> mElevator.set(ControlMode.PercentOutput, elevatorSpeed.getDouble(0.0))));
        SmartDashboard.putData("Lower Elevator Level", new LowerElevatorLevelCommand(getInstance()));
        SmartDashboard.putData("Raise Elevator Level", new RaiseElevatorLevelCommand(getInstance()));
        SmartDashboard.putData("Run Climber", new RunClimber(getInstance()));
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


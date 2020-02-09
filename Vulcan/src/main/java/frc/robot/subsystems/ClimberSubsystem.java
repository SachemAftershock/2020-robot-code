package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase implements SubsystemInterface {

    private static ClimberSubsystem mInstance;

    private final WPI_TalonSRX mElevator, mLifter;

    private ClimbElevatorPosition mCurrentPosition, mDesiredPosition;
    
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
    }

    @Override
    public void init() {
        mElevator.setSelectedSensorPosition(0); //Stowed Position
    }

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

    public boolean isAtDesiredPosition() {
        final boolean atDesiredPosition = Math.abs(mElevator.getSelectedSensorPosition() - mDesiredPosition.getEncoderCount()) <= ClimberConstants.kEpsilon;
        if(atDesiredPosition) {
            mCurrentPosition = mDesiredPosition;
        }
        return atDesiredPosition;
    }

    public void driveLifter() {
        mLifter.set(ControlMode.PercentOutput, ClimberConstants.kLifterSpeed);
    }
    public void stopLifter() {
        mLifter.set(ControlMode.PercentOutput, 0.0);
    }

    public enum ClimbElevatorPosition {
        eLow(0), eMid(4000), eHigh(8000); //TODO:Find values

        private final int mEncoderCount;

        private ClimbElevatorPosition(int encoderCount) {
            mEncoderCount = encoderCount;
        }

        public int getEncoderCount() {
            return mEncoderCount;
        }
    }

    public synchronized static ClimberSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ClimberSubsystem();
        }
        return mInstance;
    }
}


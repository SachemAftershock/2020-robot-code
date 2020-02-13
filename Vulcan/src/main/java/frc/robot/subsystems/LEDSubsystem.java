package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

/**
 * RoboRIO Master Code to process appropriate LED Color & sends data over I2C to Arduino LED Slave
 * @author Shreyas Prasad
 */
public class LEDSubsystem extends SubsystemBase implements SubsystemInterface {

    private static LEDSubsystem mInstance;

	private final Timer mTimer;
    private final I2C mI2c;
	public SystemState mCurrentMode, mDesiredMode, mPrevMode;
	private boolean mFirstRun, mForcedCommandRunning;

    private LEDSubsystem() {
		mTimer = new Timer();
        mI2c = new I2C(I2C.Port.kOnboard, LEDConstants.kArduinoI2CAddress);

		mCurrentMode = SystemState.eIdle;
		mDesiredMode = SystemState.eIdle;
		mPrevMode = SystemState.eIdle;
		mFirstRun = true;
		mForcedCommandRunning = false;
    }

    @Override
    public void init() {
        
    }

    @Override
    public void periodic() {
		if(!mForcedCommandRunning) {
			final boolean turretAimed = TurretSubsystem.getInstance().isAimedAtTarget();
			final boolean shooterAtRPM = ShooterSubsystem.getInstance().isAtTargetRPM();
			if(DriverStation.getInstance().isAutonomous()) {
				mDesiredMode = SystemState.eAuton;
			} else if(ClimberSubsystem.getInstance().isClimbing()) {
				mDesiredMode = SystemState.eClimbing;
			} else if(ClimberSubsystem.getInstance().predictIsClimbFinished()) {
				mDesiredMode = SystemState.eFinishedClimb;
			} else if(WheelControllerSubsystem.getInstance().isRunning()) {
				mDesiredMode = SystemState.eWheelControlRunning;
			} else if(CollisionAvoidanceSubsystem.getInstance().getSlowdownScaleFactor() != 1.0) {
				mDesiredMode = SystemState.eCollisionAvoidanceRunning;
			} else if(SuperstructureSubsystem.getInstance().isShotAuthorized() && turretAimed && shooterAtRPM) {
				mDesiredMode = SystemState.eShooting;
			} else if(turretAimed && shooterAtRPM) {
				mDesiredMode = SystemState.eReadyToShoot;
			} else if(DriveSubsystem.getInstance().isPrecisionMode()) {
				if(DriveSubsystem.getInstance().isHighGear()) {
					mDesiredMode = SystemState.eHighGearPrecision;
				} else {
					mDesiredMode = SystemState.eLowGearPrecision;
				}
			} else if(!TurretSubsystem.getInstance().isAutoTargetingEnabled()) {
				mDesiredMode = SystemState.eOverrideToggle;
			} else if(SuperstructureSubsystem.getInstance().getCurrentMode() == SuperstructureMode.eArmed) {
				mDesiredMode = SystemState.eArmedMode;
			} else if(SuperstructureSubsystem.getInstance().getCurrentMode() == SuperstructureMode.eFeed) {
				mDesiredMode = SystemState.eFeedMode;
			} else {
				mDesiredMode = SystemState.eIdle;
			}
		}
		
		if(mCurrentMode.getLEDMode() != mDesiredMode.getLEDMode()) {
			processModeChange();
		}
		mPrevMode = mCurrentMode;
	}

	public void forceSystemState(SystemState state) {
		mDesiredMode = state;
		mForcedCommandRunning = true;
	}
	
	private void processModeChange() {
		final LEDMode desiredLEDMode = mDesiredMode.getLEDMode();
		switch(desiredLEDMode) { //TODO: Get all desired LED configs in
			case eOff:
			case eRainbow:
			case eBlue:
			case eRed:
				setColor(desiredLEDMode);
				mCurrentMode = mDesiredMode;
				mForcedCommandRunning = false;
				break;
			case eBulletAmber:
				if(mFirstRun) {
					mTimer.start();
					mPrevMode = mCurrentMode;
					setColor(desiredLEDMode);
					mFirstRun = false;
				}
				if(mTimer.get() >= LEDConstants.kBulletRunTime) {
					mTimer.stop();
					setColor(mPrevMode.getLEDMode());
					mFirstRun = true;
					mCurrentMode = mDesiredMode;
					mForcedCommandRunning = false;
				}
				break;
		}
	}

	/**
	 * Enum to Drive Logical System Mode of the LED Strip
	 * 
	 * @author Shreyas Prasad
	 */
	public enum SystemState {
		eInit(LEDMode.eBulletAmber), eIdle(LEDMode.eOff), eAuton(LEDMode.eRainbow), eOverrideToggle(LEDMode.ePink),
		eArmedMode(LEDMode.eTeal), eFeedMode(LEDMode.eBlue), eExpel(LEDMode.eTeal), eReadyToShoot(LEDMode.eBlink), eShooting(LEDMode.eBulletAmber),
		eHighGearPrecision(LEDMode.eRedYellowAlternate), eLowGearPrecision(LEDMode.eBlueYellowAlternate),
		eClimbing(LEDMode.eBulletAmber), eFinishedClimb(LEDMode.eRainbow), eWheelControlRunning(LEDMode.eColorCycle),
		eCollisionAvoidanceRunning(LEDMode.ePurple);
		
		private final LEDMode mLEDMode;

		private SystemState(LEDMode ledMode) {
			mLEDMode = ledMode;
		}

		public LEDMode getLEDMode() {
			return mLEDMode;
		}
    }
    
    /**
     * Enum to control current state of LED Strip
     * 
	 * @author Shreyas Prasad
     */
    private enum LEDMode { //TODO: Get Correct bytes
		eOff((byte)'n'), eRed((byte)'r'), eRedYellowAlternate((byte)'r'), eBlue((byte)'b'), eBlueYellowAlternate((byte)'b'), eGreen((byte)'g'), ePink((byte)'p'), 
		eTeal((byte)'t'), eRainbow((byte)'a'), eBlink((byte)'e'), eBulletAmber((byte)'l'), eColorCycle((byte)'j'),
		ePurple((byte)'p');

		private final byte mArduinoCommand;

		private LEDMode(byte arduinoCommand) {
			mArduinoCommand = arduinoCommand;
		}

		public byte getArduinoCommand() {
			return mArduinoCommand;
		}
	}

	/**
	 * Send current desired LED Strip mode over I2C to Arduino
	 * 
	 * @param systemColor
	 *            SystemLEDMode enum element to set LEDStrip
	 */
	private void setColor(LEDMode color) {
		// Test to make sure color is different than current mode to avoid
		// redundancy
		final LEDMode currentLEDMode = mCurrentMode.getLEDMode();
		if (!currentLEDMode.equals(color)) {
			final byte arduinoCommand = color.getArduinoCommand();
			mI2c.writeBulk(new byte[] { arduinoCommand });
			mCurrentMode = mDesiredMode;
		}
    }
    
    @Override
    public void outputTelemetry() {
        
    }

    public synchronized static LEDSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new LEDSubsystem();
        }
        return mInstance;
    }
}
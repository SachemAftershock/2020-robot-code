package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.SuperstructureSubsystem.SuperstructureMode;

/**
 * RoboRIO Master Code to process appropriate LED Color & send data over I2C to Arduino LED Slave
 * 
 * @author Shreyas Prasad
 */
public class LEDSubsystem extends SubsystemBase implements SubsystemInterface {

    private static LEDSubsystem mInstance;

	private final Timer mTimer;
    private final I2C mI2c;
	public SystemState mCurrentMode, mDesiredMode, mPrevMode;
	private boolean mFirstRun, mForcedCommandRunning;

	/**
	 * Constructor for LEDSubsystem Class
	 */
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

	/**
	 * LED Subsystem State Machine
	 */
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
			//TODO: After driving a bit, determine if we should have two lists for two arduinos for two seperate LED Strips
		}
		
		if(mCurrentMode.getLEDMode() != mDesiredMode.getLEDMode()) {
			processModeChange();
		}
		mPrevMode = mCurrentMode;
	}

	/**
	 * Forcefully changes the LED State & overrides the Robot State Check Processor
	 * 
	 * @param state SystemState to change the LEDs to
	 */
	public void forceSystemState(SystemState state) {
		mDesiredMode = state;
		mForcedCommandRunning = true;
	}
	
	/**
	 * Processes change in current and desired LED Mode
	 */
	private void processModeChange() {
		final LEDMode desiredLEDMode = mDesiredMode.getLEDMode();
		switch(desiredLEDMode) {
			case eOff:
			case eRainbow:
			case eBlue:
			case eRed:
			case ePink:
			case ePurple:
			case eTeal:
			case eWhite:
			case eYellow:
			case eGreen:
			case eOrange:
			case eColorCycle:
				setColor(desiredLEDMode);
				mCurrentMode = mDesiredMode;
				mForcedCommandRunning = false;
				break;

			case eWhiteBlink:
			case eYellowBlink:
			case eBlueBlink:
			case eRedBlink:
			case eTealBullet:
			case eBulletOrange:
			case eBulletRed:
			case eBulletWhite:
			case eBlueRedAlternate:
			case eBlueYellowAlternate:
			case eRedYellowAlternate:
				if(mFirstRun) {
					mTimer.start();
					mPrevMode = mCurrentMode;
					setColor(desiredLEDMode);
					mFirstRun = false;
				}
				if(mTimer.get() >= LEDConstants.kTempModeRunTime) {
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
		eInit(LEDMode.eWhiteBlink), eIdle(LEDMode.eOff), eAuton(LEDMode.eRainbow), eOverrideToggle(LEDMode.ePink),
		eArmedMode(LEDMode.eRed), eFeedMode(LEDMode.eBlue), eExpel(LEDMode.eTealBullet), eReadyToShoot(LEDMode.eYellowBlink), eShooting(LEDMode.eBulletRed),
		eHighGearPrecision(LEDMode.eRedYellowAlternate), eLowGearPrecision(LEDMode.eBlueYellowAlternate),
		eClimbing(LEDMode.eBulletWhite), eFinishedClimb(LEDMode.eRainbow), eWheelControlRunning(LEDMode.eColorCycle),
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
    private enum LEDMode {
		eOff((byte)'n'), eRed((byte)'r'), eRedYellowAlternate((byte)'x'), eBlue((byte)'b'), eBlueRedAlternate((byte)'s'), eOrange((byte)'m'), eRedBlink((byte)'f'), eBlueBlink((byte)'h'),
		eWhite((byte)'w'), eGreen((byte)'g'), ePink((byte)'p'), eYellow((byte)'y'), eYellowBlink((byte)'j'), eTeal((byte)'t'), eTealBullet((byte)'d'), eRainbow((byte)'a'), eWhiteBlink((byte)'o'), eBulletOrange((byte)'l'), eColorCycle((byte)'c'),
		eBlueYellowAlternate((byte)'v'), ePurple((byte)'u'), eBulletRed((byte)'e'), eBulletWhite((byte)'i');

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
	 * @param systemColor SystemLEDMode enum element to set LEDStrip
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
		SmartDashboard.putData(getInstance());
	}
	
	@Override
	public void runTest() {
		// TODO Auto-generated method stub
	}

	/**
	 * @return LEDSubsystem Singleton Instance
	 */
    public synchronized static LEDSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new LEDSubsystem();
        }
        return mInstance;
    }
}
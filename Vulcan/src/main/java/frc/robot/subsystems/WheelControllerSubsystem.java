package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.WheelControllerConstants;

/**
 * Control Panel Wheel Controller Subsystem
 * 
 * @author Shreyas Prasad
 */
public class WheelControllerSubsystem extends SubsystemBase implements SubsystemInterface {

    private static WheelControllerSubsystem mInstance;

    private final WPI_VictorSPX mWheelSpinner;
    private final ColorSensorV3 mColorSensor;
    private final DoubleSolenoid mExtender;
    private String mTargetColorString;
    private ColorLUT mFieldRelativeTargetColor, mRobotRelativeTargetColor, mCurrentDetectedColor, mStartColor;
    private int mTimesPositionedToStartColor, mNumberOfWedgesCrossed, mWedgesRequiredToRotate;
    private boolean mPreviousExists, mCommandRunning;
    private final CircularColorArray mColors;

    /**
     * Constructor for WheelControllerSubsystem Class
     */
    private WheelControllerSubsystem() {

        mWheelSpinner = new WPI_VictorSPX(WheelControllerConstants.kWheelControllerId);
        addChild("Wheel Spinner", mWheelSpinner);

        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mExtender = new DoubleSolenoid(PneumaticConstants.kPcmId, WheelControllerConstants.kExtenderForwardId, WheelControllerConstants.kExtenderReverseId);
        addChild("Extender", mExtender);
        mExtender.set(Value.kReverse);

        mColors = new CircularColorArray();

        mFieldRelativeTargetColor = ColorLUT.eUnknown;
        mRobotRelativeTargetColor = ColorLUT.eUnknown;
        mCurrentDetectedColor = ColorLUT.eUnknown;
        mStartColor = ColorLUT.eUnknown;

        mTimesPositionedToStartColor = 0;
        mPreviousExists = false;

        mTargetColorString = "";

        mCommandRunning = false;
    }

    @Override
    public void init() {
        mExtender.set(Value.kReverse);
        mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Checks for current detected color & for when the field gives Robots a target color
     */
    @Override
    public void periodic() {
        ColorLUT mPreviousColor = mCurrentDetectedColor;
        if(!mPreviousExists) {
            mPreviousExists = mPreviousColor != ColorLUT.eUnknown;
        }
        mCurrentDetectedColor = getCurrentColor();

        if(mTargetColorString.equals("")) {
            mTargetColorString = DriverStation.getInstance().getGameSpecificMessage();
            if(!mTargetColorString.equals("")) {
                switch(mTargetColorString.charAt(0)) {
                    case 'B':
                        mFieldRelativeTargetColor = ColorLUT.eBlue;
                        break;
                    case 'Y':
                        mFieldRelativeTargetColor = ColorLUT.eYellow;
                        break;
                    case 'R':
                        mFieldRelativeTargetColor = ColorLUT.eRed;
                        break;
                    case 'G':
                        mFieldRelativeTargetColor = ColorLUT.eGreen;
                        break;
                    default:
                        DriverStation.reportError("ERROR: TARGET COLOR NOT DETECTED", false);
                        break;
                }
                mRobotRelativeTargetColor = mColors.getColor(mColors.getIndex(mFieldRelativeTargetColor) + WheelControllerConstants.kRobotReadingVarianceIndex);
            } else {
                mTargetColorString = "";
            }
        }

        if(mStartColor != ColorLUT.eUnknown && mPreviousExists) {
            if(mCurrentDetectedColor != mPreviousColor && mCurrentDetectedColor == mStartColor) {
                mTimesPositionedToStartColor++;
            }
            if(mCurrentDetectedColor != mPreviousColor && mPreviousColor != ColorLUT.eUnknown) {
                mNumberOfWedgesCrossed++;
            }
        }
    }

    /**
     * Starts Automatic Rotational Control (Spins Control Panel 3.5 Times)
     */
    public void startRotationControl() {
        mCommandRunning = true;
        mPreviousExists = false;
        mStartColor = mCurrentDetectedColor;
        mTimesPositionedToStartColor = 0;
        if(mStartColor == ColorLUT.eUnknown) {
            for(int i = 0; i < 5000 / 20; i++) {
                mStartColor = mCurrentDetectedColor;
                if(mStartColor != ColorLUT.eUnknown) {
                    break;
                }
            }
            if(mStartColor == ColorLUT.eUnknown) {
                DriverStation.reportError("ERROR: COLOR UNKNOWN", false);
            }
        }
        mWheelSpinner.set(ControlMode.PercentOutput, WheelControllerConstants.kWheelSpinSpeed);
    }

    /**
     * Stops Wheel Controller Motor to end Rotational Control
     */
    public void endRotationControl() {
        mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
        mCommandRunning = false;
    }

    /**
     * Gets if the Automatic Rotational Process should end
     * 
     * @return <i> true </i> if Control Panel has rotated 3.5 times; <i> false </i> otherwise
     */
    public boolean isRotateConditionMet() {
        return mTimesPositionedToStartColor >= 7;
    }

    /**
     * Begins Color Targeting
     */
    public void startColorTargeting() {
        mCommandRunning = true;
        mPreviousExists = false;
        CalculatedSpin calculatedSpin = calculateSpinDirection();
        mWedgesRequiredToRotate = calculatedSpin.getWedgesToRotate();
        mStartColor = mCurrentDetectedColor;
        mNumberOfWedgesCrossed = 0;
        if(calculatedSpin.getTargetDirection() == Direction.FORWARD) {
            mWheelSpinner.set(ControlMode.PercentOutput, WheelControllerConstants.kWheelSpinSpeed);
        } else if(calculatedSpin.getTargetDirection() == Direction.REVERSE) {
            mWheelSpinner.set(ControlMode.PercentOutput, -WheelControllerConstants.kWheelSpinSpeed);
        } else {
            mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
            DriverStation.reportWarning("WARNING: ALREADY ON TARGET COLOR, SPEED SET TO 0", false);
            //If this isn't true, fix todo in calculateSpinDirection()
        }
    }

    /**
     * Stops Wheel Controller Motor to end Color Targeting
     */
    public void stopColorTargeting() {
        mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
        mCommandRunning = false;
    }

    /**
     * Gets if the Magnitude of Wedges Crossed is equal to the number required as calculated by {@link WheelControllerSubsystem#calculateSpinDirection()}
     * 
     * @return <i> true </i> if the Target Color has been found; <i> false </i> otherwise
     */
    public boolean isTargetColorFound() {
        return mNumberOfWedgesCrossed == mWedgesRequiredToRotate;
    }

    public void deployExtender() {
        mExtender.set(Value.kForward);
    }

    public void retractExtender() {
        if(CollisionAvoidanceSubsystem.getInstance().getUltrasonicDistanceInches() >= WheelControllerConstants.kWheelExtenderLimitMaxDistanceIn) {
            mExtender.set(Value.kReverse);
        }
    }

    public boolean isExtended() {
        return mExtender.get() == Value.kForward;
    }

    /**
     * Calculates Smallest Vector to Rotate to Required Color Target
     * 
     * @return CalculatedSpin Vector, containing Magnitude and Direction to Spin
     */
    private CalculatedSpin calculateSpinDirection() {
        Direction targetDirection;
        int wedgesToRotate;
        int colorVariance = mColors.getIndex(mRobotRelativeTargetColor) - mColors.getIndex(mCurrentDetectedColor);
        switch(colorVariance) {
            case 3:
            case -1:
                targetDirection = Direction.FORWARD;
                wedgesToRotate = 1;
                break;
            case -3:
            case 1:
                targetDirection = Direction.REVERSE;
                wedgesToRotate = 1;
                break;
            case -2:
            case 2:
                targetDirection = Direction.FORWARD;
                wedgesToRotate = 2;
                break;
            default:
                targetDirection = Direction.NONE; //If below is changed, must change direction
                wedgesToRotate = 0; //TODO: Find out if we need to rotate wheel if target color == current color, if so set this to 4
                break;
        }
        return (new CalculatedSpin(targetDirection, wedgesToRotate));
    }

    /**
     * Gets the current Color as detected by the Color Sensor
     * 
     * @return the current color detected by the Color Sensor
     */
    private ColorLUT getCurrentColor() {
        Color mColor = mColorSensor.getColor();
        if(isColor(mColor, ColorLUT.eBlue)) {
            return ColorLUT.eBlue;
        } else if(isColor(mColor, ColorLUT.eYellow)) {
            return ColorLUT.eYellow;
        } else if(isColor(mColor, ColorLUT.eRed)) {
            return ColorLUT.eRed;
        } else if(isColor(mColor, ColorLUT.eGreen)) {
            return ColorLUT.eGreen;
        } else {
            return ColorLUT.eUnknown;
        }
    }

    /**
     * Checks if the Color Sensor detected color is equal to the calibrated Control Panel Colors
     * 
     * @param detectedColor color detected by Color Sensor
     * 
     * @param calibratedColor color to check the detected color against
     * 
     * @return <i> true </i> if the detected color is the calibrated color; <i> false </i> otherwise
     */
    private boolean isColor(Color detectedColor, ColorLUT calibratedColor) {
        return withinRange(detectedColor.red, calibratedColor.getRed()) && 
            withinRange(detectedColor.blue, calibratedColor.getBlue()) &&
            withinRange(detectedColor.green, calibratedColor.getGreen());
    }

    /**
     * Tests if input parameters are within Color Tolerance
     * 
     * @param a First Single Color Value
     * 
     * @param b Second Single Color Value
     * 
     * @return If a and b are within the accepted tolerance
     */
    private boolean withinRange(double a, double b) {
        return Math.abs(a - b) <= WheelControllerConstants.kColorTolerance;
    }

    /**
     * Gets if the target color has been given by the FMS
     * 
     * @return <i> true </i> if the FMS has given the target color;<i> false </i> otherwise
     */
    public boolean isTargetColorKnown() {
        return mFieldRelativeTargetColor != null;
    }

    public void manualControl(double pow) {
        pow *= WheelControllerConstants.kManualWheelProportion;
        mWheelSpinner.set(ControlMode.PercentOutput, pow);
    }

    public boolean isCommandRunning() {
        return mCommandRunning;
    }

    /**
     * Lookup-Table consisting of the RGB values of the Control Panel Colors
     * 
     * @author Shreyas Prasad
     */
    private enum ColorLUT {
        eBlue(0.129, 0.429, 0.441), eYellow(0.314, 0.564, 0.120), eRed(0.462, 0.381, 0.157), eGreen(0.167, 0.581, 0.250), eUnknown(0,0,0);

        private final double mRed, mGreen, mBlue;

        ColorLUT(double red, double green, double blue) {
            mRed = red;
            mGreen = green;
            mBlue = blue;
        }

        public double getRed() {
            return mRed;
        }
        public double getBlue(){
            return mBlue;
        }
        public double getGreen(){
            return mGreen;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }

    /**
     * Direction to Spin Control Panel
     */
    private enum Direction {
        FORWARD, REVERSE, NONE;
    }

    /**
     * Vector to Spin the Control Panel in Positional(Color) Control
     * 
     * @author Shreyas Prasad
     */
    private class CalculatedSpin {
        private Direction mTargetDirection;
        private int mWedgesToRotate;

        public CalculatedSpin(Direction targetDirection, int wedgesToRotate) {
            mTargetDirection = targetDirection;
            mWedgesToRotate = wedgesToRotate;
        }

        /**
         * Gets Direction Component of Vector
         * 
         * @return the direction to get to the target color in the shortest time
         */
        public Direction getTargetDirection() {
            return mTargetDirection;
        }

        /**
         * Gets Magnitude of Vector in terms of Control Panel Color Wedges to rotate
         * 
         * @return the number of wedges needed to rotate to reach the target
         */
        public int getWedgesToRotate() {
            return mWedgesToRotate;
        }
    }

    /**
     * Circular Array representing the cycle of colors on the Control Panel
     * 
     * @author Shreyas Prasad
     */
    private class CircularColorArray {

        private ColorLUT[] mCircularArray;
        private final int kListSize = 4;
        
        /**
         * Constructor for Circular Array Class
         * <p>
         * Fills Array with Control Panel Colors in <b>Blue, Yellow, Red, Green</b> order
         */
        public CircularColorArray() {
            mCircularArray = new ColorLUT[kListSize];

            mCircularArray[0] = ColorLUT.eBlue;
            mCircularArray[1] = ColorLUT.eYellow;
            mCircularArray[2] = ColorLUT.eRed;
            mCircularArray[3] = ColorLUT.eGreen;
        }

        /**
         * Gets color at the index; wraps around if index is outside of [0,3]
         * 
         * @param index the index to get the color from
         * 
         * @return Color found at the normalized index
         */
        public ColorLUT getColor(int index) {
            if(index < 0) {
                index += kListSize;
            }
            return mCircularArray[index % 4];
        }

        /**
         * Gets index from [0,3] of the selected Color
         * 
         * @param targetColor the color to search the Circular Array for
         * 
         * @return the index from [0,3] of the selected color in the Circular Array
         */
        public int getIndex(ColorLUT targetColor) {
            for(int i = 0; i < kListSize; i++) {
                if(targetColor == mCircularArray[i]) {
                    return i;
                }
            }
            DriverStation.reportError("ERROR: COLOR NOT FOUND IN CIRCULAR ARRAY", false);
            return 0;
        }
    }

    /**
     * Whether or not the Wheel Controller is active
     * <p>
     * Used for {@link LEDSubsystem#periodic() LED State Processor}
     * 
     * @return <i> true </i> if Wheel Spinner Motor is running; <i> false </i> otherwise
     */
    public boolean isRunning() {
        return Math.abs(mWheelSpinner.get()) > 0;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        SmartDashboard.putString("Current Detected Color", mCurrentDetectedColor.toString());
        SmartDashboard.putString("Field Relative Target Color", mFieldRelativeTargetColor.toString());
        SmartDashboard.putString("Robot Relative Target Color", mRobotRelativeTargetColor.toString());
        SmartDashboard.putBoolean("Is Extender Deployed", mExtender.get() == Value.kForward);
        SmartDashboard.putNumber("Times Pos to Start", mTimesPositionedToStartColor);
        SmartDashboard.putNumber("# of Wedges Crossed", mNumberOfWedgesCrossed);
        SmartDashboard.putNumber("Req Wedges to Rot", mWedgesRequiredToRotate);
        SmartDashboard.putData(mWheelSpinner);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * @return WheelControllerSubsystem Singleton Instance
     */
    public synchronized static WheelControllerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new WheelControllerSubsystem();
        }
        return mInstance;
    }
}
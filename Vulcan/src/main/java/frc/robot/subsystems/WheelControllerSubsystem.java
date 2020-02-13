package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WheelControllerConstants;

/**
 * Control Panel Wheel Controller Subsystem
 * @author Shreyas Prasad
 */
public class WheelControllerSubsystem extends SubsystemBase implements SubsystemInterface {

    private static WheelControllerSubsystem mInstance;

    private final WPI_TalonSRX mWheelSpinner;
    private final ColorSensorV3 mColorSensor;
    private final DoubleSolenoid mExtender;
    private String mTargetColorString;
    private ColorLUT mFieldRelativeTargetColor, mRobotRelativeTargetColor, mCurrentDetectedColor, mStartColor;
    private int mTimesPositionedToStartColor, mNumberOfWedgesCrossed, mWedgesRequiredToRotate;
    private boolean mPreviousExists;
    private final CircularColorArray mColors;

    private WheelControllerSubsystem() {

        mWheelSpinner = new WPI_TalonSRX(WheelControllerConstants.kWheelControllerId);

        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mExtender = new DoubleSolenoid(Constants.kPcmBId, WheelControllerConstants.kExtenderForwardId, WheelControllerConstants.kExtenderReverseId);
        mExtender.set(Value.kReverse);

        mColors = new CircularColorArray();

        mFieldRelativeTargetColor = ColorLUT.eUnknown;
        mRobotRelativeTargetColor = ColorLUT.eUnknown;
        mCurrentDetectedColor = ColorLUT.eUnknown;
        mStartColor = ColorLUT.eUnknown;

        mTimesPositionedToStartColor = 0;
        mPreviousExists = false;
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {
        ColorLUT mPreviousColor = mCurrentDetectedColor;
        if(!mPreviousExists)
            mPreviousExists = mPreviousColor != ColorLUT.eUnknown;
        mCurrentDetectedColor = getCurrentColor();
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

    public void startPositionControl() {
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

    public void endPositionControl() {
        mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean isPositionConditionMet() {
        //3.5 Turns
        return mTimesPositionedToStartColor >= 7;
    }

    public void turnToTargetColor() {
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

    public void stopColorTargeting() {
        mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
    }

    public boolean isTargetColorFound() {
        return mNumberOfWedgesCrossed == mWedgesRequiredToRotate;
    }

    public void toggleExtender() {
        switch (mExtender.get()) {
            case kForward : 
                mExtender.set(Value.kReverse);
                CollisionAvoidanceSubsystem.getInstance().setStandardStandoff();
                break;
            case kReverse : 
                mExtender.set(Value.kForward);
                CollisionAvoidanceSubsystem.getInstance().setColorWheelStandoff();
                break;
            default :
                mExtender.set(Value.kReverse);
                CollisionAvoidanceSubsystem.getInstance().setStandardStandoff();
                DriverStation.reportError("ERROR: COLOR WHEEL EXTENDER VALUE INVALID", false);
                break;
        }
    }

    /**
     * Calculates Smallest Vector to Rotate to Required Color Target
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

    private boolean isColor(Color detectedColor, ColorLUT calibratedColor) {
        return withinRange(detectedColor.red, calibratedColor.getRed()) && 
            withinRange(detectedColor.blue, calibratedColor.getBlue()) &&
            withinRange(detectedColor.green, calibratedColor.getGreen());
    }

    /**
     * Tests if input parameters are within Color Tolerance
     * @param a First Single Color Value
     * @param b Second Single Color Value
     * @return If a and b are within the accepted tolerance
     */
    private boolean withinRange(double a, double b) {
        return Math.abs(a - b) <= WheelControllerConstants.kColorTolerance;
    }

    public boolean isTargetColorKnown() {
        return mFieldRelativeTargetColor != null;
    }

    /**
     * Lookup-Table consisting of the RGB values of the Control Panel Colors
     * @author Shreyas Prasad
     */
    private enum ColorLUT implements Sendable {
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
        public void initSendable(SendableBuilder builder) {}
    }

    /**
     * Direction to Spin Control Panel
     */
    private enum Direction {
        FORWARD, REVERSE, NONE;
    }

    /**
     * Vector to Spin the Control Panel in Positional(Color) Control
     * @author Shreyas Prasad
     */
    private class CalculatedSpin {
        private Direction mTargetDirection;
        private int mWedgesToRotate;

        public CalculatedSpin(Direction targetDirection, int wedgesToRotate) {
            mTargetDirection = targetDirection;
            mWedgesToRotate = wedgesToRotate;
        }

        public Direction getTargetDirection() {
            return mTargetDirection;
        }

        public int getWedgesToRotate() {
            return mWedgesToRotate;
        }
    }

    /**
     * Circular Array representing the cycle of colors on the Control Pnale
     * @author Shreyas Prasad
     */
    private class CircularColorArray {

        private ColorLUT[] mCircularArray;
        private final int kListSize = 4;
    
        public CircularColorArray() {
            mCircularArray = new ColorLUT[kListSize];

            mCircularArray[0] = ColorLUT.eBlue;
            mCircularArray[1] = ColorLUT.eYellow;
            mCircularArray[2] = ColorLUT.eRed;
            mCircularArray[3] = ColorLUT.eGreen;
        }

        public ColorLUT getColor(int index) {
            if(index < 0) {
                index += kListSize;
            }
            return mCircularArray[index % 4];
        }

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

    public boolean isRunning() {
        return Math.abs(mWheelSpinner.get()) > 0;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData("Current Detected Color", mCurrentDetectedColor);
        SmartDashboard.putData("Field Relative Target Color", mFieldRelativeTargetColor);
        SmartDashboard.putData("Robot Relative Target Color", mRobotRelativeTargetColor);
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
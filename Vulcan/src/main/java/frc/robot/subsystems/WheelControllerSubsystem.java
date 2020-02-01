package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelControllerSubsystem extends SubsystemBase {

    private static WheelControllerSubsystem mInstance;

    private final TalonSRX mWheelSpinner;
    private final ColorSensorV3 mColorSensor;
    private final AnalogInput mUltrasonic;
    private final DoubleSolenoid mExtender;
    private String mTargetColorString;
    private ColorLUT mFieldRelativeTargetColor, mRobotRelativeTargetColor, mCurrentDetectedColor, mStartColor;
    private int mTimesPositionedToStartColor, mNumberOfWedgesCrossed, mWedgesRequiredToRotate;
    private boolean mPreviousExists;
    private final CircularColorArray mColors;

    // Expected to read color 90deg from the field color sensor, therefore 2 45deg color wedges over
    private final int kRobotReadingVarianceIndex = 2;
    private final double kWheelSpinSpeed = 0.4;


    public WheelControllerSubsystem() {

        mWheelSpinner = new TalonSRX(Constants.kWheelControllerId);

        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mExtender = new DoubleSolenoid(Constants.kPcmAId, Constants.kExtenderForwardId, Constants.kExtenderReverseId);
        mExtender.set(Value.kReverse);

        mColors = new CircularColorArray();

        mUltrasonic = new AnalogInput(Constants.kUltrasonicId);

        mFieldRelativeTargetColor = ColorLUT.eUnknown;
        mRobotRelativeTargetColor = ColorLUT.eUnknown;
        mCurrentDetectedColor = ColorLUT.eUnknown;
        mStartColor = ColorLUT.eUnknown;

        mTimesPositionedToStartColor = 0;
        mPreviousExists = false;
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
            mRobotRelativeTargetColor = mColors.getColor(mColors.getIndex(mFieldRelativeTargetColor) + kRobotReadingVarianceIndex);
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
            DriverStation.reportError("ERROR: COLOR UNKNOWN", false);
            //TODO: Add a loop to retry and timeout here if needed
        }
        mWheelSpinner.set(ControlMode.PercentOutput, kWheelSpinSpeed);
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
            mWheelSpinner.set(ControlMode.PercentOutput, kWheelSpinSpeed);
        } else if(calculatedSpin.getTargetDirection() == Direction.REVERSE) {
            mWheelSpinner.set(ControlMode.PercentOutput, -kWheelSpinSpeed);
        } else {
            mWheelSpinner.set(ControlMode.PercentOutput, 0.0);
            DriverStation.reportWarning("WARNING: ALREADY ON TARGET COLOR, SPEED SET TO 0", false);
            //If this isn't true, fix TODO in calculateSpinDirection()
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

    public CalculatedSpin calculateSpinDirection() {
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

    public ColorLUT getCurrentColor() {
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
    private boolean withinRange(double a, double b) {
        final double kTolerance = 0.08;
        return Math.abs(a - b) <= kTolerance;
    }

    public boolean isTargetColorKnown() {
        return mFieldRelativeTargetColor != null;
    }

    //TODO: Have to implement safe driving features to use
    private double getUltrasonicDistance() {
        return mUltrasonic.getValue() * Constants.kUltrasonicValueToInches;
    }

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
    }

    private enum Direction {
        FORWARD, REVERSE, NONE;
    }

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

    public synchronized static WheelControllerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new WheelControllerSubsystem();
        }
        return mInstance;
    }
}
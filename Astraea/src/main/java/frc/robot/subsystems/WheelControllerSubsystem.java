package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelControllerSubsystem extends SubsystemBase {

    private final TalonSRX mWheelSpinner;
    private final ColorSensorV3 mColorSensor;
    private final AnalogInput mUltrasonic;
    private final DoubleSolenoid mExtender;
    private String mTargetColorString;
    private ColorLUT mFieldRelativeTargetColor, mRobotRelativeTargetColor, mCurrentDetectedColor, mStartColor;
    private int mTimesPositionedToStartColor, mNumberOfWedgesCrossed, mWedgesRequiredToRotate;
    private boolean mPreviousExists;
    private final CircularColorArray mColors;
    private final double kDurationPerWedgeInSeconds = 0.5; //TODO: Time this value correctly
    private final double kUltrasonicValueToInches = 0.125;
    private final double kWheelSpinSpeed = 0.4;

    public WheelControllerSubsystem() {

        mWheelSpinner = new TalonSRX(Constants.kWheelControllerId);

        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mExtender = new DoubleSolenoid(Constants.kPcmId, Constants.kExtenderForwardId, Constants.kExtenderReverseId);
        mExtender.set(Value.kReverse);

        mColors = new CircularColorArray();

        mUltrasonic = new AnalogInput(Constants.kUltrasonicId);

        mFieldRelativeTargetColor = ColorLUT.UNKNOWN;
        mRobotRelativeTargetColor = ColorLUT.UNKNOWN;
        mCurrentDetectedColor = ColorLUT.UNKNOWN;
        mStartColor = ColorLUT.UNKNOWN;

        mTimesPositionedToStartColor = 0;
        mPreviousExists = false;
    }

    @Override
    public void periodic() {
        ColorLUT mPreviousColor = mCurrentDetectedColor;
        if(!mPreviousExists)
            mPreviousExists = mPreviousColor != ColorLUT.UNKNOWN;
        mCurrentDetectedColor = getCurrentColor();
        mTargetColorString = DriverStation.getInstance().getGameSpecificMessage();
        if(!mTargetColorString.equals("")) {
            switch(mTargetColorString.charAt(0)) {
                case 'B':
                    mFieldRelativeTargetColor = ColorLUT.BLUE;
                    break;
                case 'Y':
                    mFieldRelativeTargetColor = ColorLUT.YELLOW;
                    break;
                case 'R':
                    mFieldRelativeTargetColor = ColorLUT.RED;
                    break;
                case 'G':
                    mFieldRelativeTargetColor = ColorLUT.GREEN;
                    break;
                default:
                    System.out.println("ERROR: TARGET COLOR NOT DETECTED");
                    break;
            }
            //TODO: Replace (2) with constant
            // Expected to read color 90deg from the field color sensor, therefore 2 45deg color wedges over
            mRobotRelativeTargetColor = mColors.getColor(mColors.getIndex(mFieldRelativeTargetColor) + 2);
        }
        if(mStartColor != ColorLUT.UNKNOWN && mPreviousExists) {
            if(mCurrentDetectedColor != mPreviousColor && mCurrentDetectedColor == mStartColor) {
                mTimesPositionedToStartColor++;
            }
            if(mCurrentDetectedColor != mPreviousColor && mPreviousColor != ColorLUT.UNKNOWN) {
                mNumberOfWedgesCrossed++;
            }
        }
        System.out.println(" COLOR: " + mCurrentDetectedColor + " POS COUNT: " + mTimesPositionedToStartColor + " COLOR COUNT: " + mNumberOfWedgesCrossed);
    }

    public void startPositionControl() {
        mPreviousExists = false;
        mStartColor = mCurrentDetectedColor;
        mTimesPositionedToStartColor = 0;
        if(mStartColor == ColorLUT.UNKNOWN) {
            System.out.println("ERROR: COLOR UNKNOWN");
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

    //TODO: Spin until Color Hit
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
            System.out.println("WARNING: ALREADY ON TARGET COLOR, SPEED SET TO 0");
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
        if(mExtender.get()  == Value.kForward) {
            mExtender.set(Value.kReverse);
        } else {
            mExtender.set(Value.kForward);
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

        /*
        if(mFieldRelativeTargetColor == Color.NONE) {
            return Direction.NONE;
        } else if(mRobotRelativeTargetColor == mColors.getColor(mColors.getIndex(mFieldRelativeTargetColor) - 2)) {
            return Direction.NONE;
        } 

        int stepsReverse = 0, stepsForward = 0;
        final int kRobotRelativeTargetIndex = mColors.getIndex(mRobotRelativeTargetColor);

        for(int i = kRobotRelativeTargetIndex; stepsReverse < 3; i--) {
            if(mColors.getColor(i) == mRobotRelativeTargetColor) {
                break;
            }
            stepsReverse++;
        }

        for(int i = kRobotRelativeTargetIndex; stepsForward < 3; i++) {
            if(mColors.getColor(i) == mRobotRelativeTargetColor) {
                break;
            }
            stepsForward++;
        }

        return stepsForward <= stepsReverse ? Direction.FORWARD : Direction.REVERSE;
        */
    }

    public ColorLUT getCurrentColor() {
        Color mColor = mColorSensor.getColor();
        //System.out.println("RED: " + mColor.red + " GREEN: " + mColor.green +  " BLUE: " + mColor.blue);
        if(isColor(mColor, ColorLUT.BLUE)) {
            return ColorLUT.BLUE;
        } else if(isColor(mColor, ColorLUT.YELLOW)) {
            return ColorLUT.YELLOW;
        } else if(isColor(mColor, ColorLUT.RED)) {
            return ColorLUT.RED;
        } else if(isColor(mColor, ColorLUT.GREEN)) {
            return ColorLUT.GREEN;
        } else {
            return ColorLUT.UNKNOWN;
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
        return mUltrasonic.getValue() * kUltrasonicValueToInches;
    }

    private enum ColorLUT {
        BLUE(0.129, 0.429, 0.441), YELLOW(0.314, 0.564, 0.120), RED(0.462, 0.381, 0.157), GREEN(0.167, 0.581, 0.250), UNKNOWN(0,0,0);

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

            mCircularArray[0] = ColorLUT.BLUE;
            mCircularArray[1] = ColorLUT.YELLOW;
            mCircularArray[2] = ColorLUT.RED;
            mCircularArray[3] = ColorLUT.GREEN;
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
            System.out.println("ERROR COLOR NOT FOUND IN CIRCULAR ARRAY");
            return 0;
        }
    }
}
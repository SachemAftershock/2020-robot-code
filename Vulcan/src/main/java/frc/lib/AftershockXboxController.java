package frc.lib;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Wrapper for XboxControllers
 * 
 * @author Shreyas Prasad
 */
public class AftershockXboxController extends XboxController {

    private final double kJoystickDeadbandTolerance = 0.15;
    private final double kTriggerDeadbandTolerance =  0.15;

    private LatchedBoolean mLeftTriggerPressed;
    private LatchedBoolean mRightTriggerPressed;

    /**
     * Constructor for Afterhock Wrapper Xbox Controllers
     * 
     * @param port Driver Station port the controller is connected to
     */
    public AftershockXboxController(final int port) {
        super(port);
    }

    /**
     * Gets X axis value on selected Joystick with deadband applied
     * 
     * @param hand Left or Right Joystick
     * 
     * @return X Axis Value of Selected Joystick with deadband applied
     */
    public double getDeadbandX(Hand hand) {
        return deadband(this.getX(hand), kJoystickDeadbandTolerance);
    }

    /**
     * Gets Y axis value on selected Joystick with deadband applied
     * 
     * @param hand Left or Right Joystick
     * 
     * @return Y Axis Value of Selected Joystick with deadband applied
     */
    public double getDeadbandY(Hand hand) {
        return deadband(this.getY(hand), kJoystickDeadbandTolerance);
    }

    /**
     * Gets if Trigger is currently held
     * 
     * @param hand Left or Right Trigger to check
     * 
     * @return If the trigger value exceeds the deadband
     */
    public boolean getTriggerHeld(Hand hand) {
        return deadband(this.getTriggerAxis(hand), kTriggerDeadbandTolerance) != 0;
    }

    /**
     * Whether the Trigger was pressed since the last check
     * 
     * @param hand Left or Right Trigger to check
     * 
     * @return Whether the Trigger was pressed since the last check
     */
    public boolean getTriggerPressed(Hand hand) {
        if(hand == Hand.kLeft) {
            return mLeftTriggerPressed.update(getTriggerHeld(hand));
        } else {
            return mRightTriggerPressed.update(getTriggerHeld(hand));
        }
    }

    /**
     * Gets if D-Pad is currently being pressed
     * 
     * @return whether POV value is not -1
     */
    public boolean getDPadActive() {
        return this.getPOV() == -1;
    }

    /**
     * Gets if Up on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 0deg
     */
    public boolean getDPadUp() {
        return this.getPOV() == POVDirection.eUp.getAngle();
    }

    /**
     * Gets if Up-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 45deg
     */
    public boolean getDPadUpRight() {
        return this.getPOV() == POVDirection.eUpRight.getAngle();
    }

    /**
     * Gets if Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 90deg
     */
    public boolean getDPadRight() {
        return this.getPOV() == POVDirection.eRight.getAngle();
    }

    /**
     * Gets if Down-Right on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 135deg
     */
    public boolean getDPadDownRight() {
        return this.getPOV() == POVDirection.eDownRight.getAngle();
    }

    /**
     * Gets if Down on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 180deg
     */
    public boolean getDPadDown() {
        return this.getPOV() == POVDirection.eDown.getAngle();
    }

    /**
     * Gets if Down-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 225deg
     */
    public boolean getDPadDownLeft() {
        return this.getPOV() == POVDirection.eDownLeft.getAngle();
    }

    /**
     * Gets if Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 270deg
     */
    public boolean getDPadLeft() {
        return this.getPOV() == POVDirection.eLeft.getAngle();
    }

    /**
     * Gets if Up-Left on the D-Pad is pressed
     * 
     * @return whether the D-Pad angle is 315deg
     */
    public boolean getDPadUpLeft() {
        return this.getPOV() == POVDirection.eUpLeft.getAngle();
    }


    /**
     * Applies a deadband to the value with given tolerance
     * 
     * @param value the value to apply the deadband to
     * 
     * @param tolerance minimum value
     * 
     * @return <b> value </b> if greater than <b> tolerance </b>; 0.0 otherwise
     */
    private double deadband(double value, double tolerance) {
        return Math.abs(value) >= tolerance ? value : 0.0;
    }

    /**
     * Lookup Table for matching direction of D-Pad on Xbox Controller pressed to an angle measure
     * 
     * @author Shreyas Prasad
     */
    private enum POVDirection {
        eUp(0), eUpRight(45), eRight(90), eDownRight(135), eDown(180),
        eDownLeft(225), eLeft(270), eUpLeft(315);

        private final int angle;

        private POVDirection(int angle) {
            this.angle = angle;
        }

        /**
         * Gets angle for D-Pad Direction Pressed
         * 
         * @return angle [0,360) corresponding to the appropriate 45deg interval on the D-Pad
         */
        private int getAngle() {
            return this.angle;
        }
    }
}
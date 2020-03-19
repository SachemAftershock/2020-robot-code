package frc.lib;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * 
 * @author Dan Waxman
 * @author Shreyas Prasad
*/
public class Limelight {
	
	private NetworkTableInstance table = null;

	private final String mTableName;

	public final static double kDefaultValue = 9999.9;

	/**
	 * Creates a new Limelight Object
	 * @param tableName The name of the Limelight's NetworkTable
	 */
	public Limelight(String tableName) {
		mTableName = tableName;
	}

	/**
	 * Light modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum LightMode {
		eOn, eOff, eBlink
	}

	/**
	 * Camera modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum CameraMode {
		eVision, eDriver
	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return getValue("tv").getDouble(kDefaultValue) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(kDefaultValue);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(kDefaultValue);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(kDefaultValue);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(kDefaultValue);
	}

	/**
	 * Gets target latency (ms).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(kDefaultValue);
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}

		return table.getTable(mTableName).getEntry(key);
	}

	public void outputTelemetry() {
		SmartDashboard.putBoolean(mTableName + " is Target", isTarget());
		SmartDashboard.putNumber(mTableName + " tx", getTx());
		SmartDashboard.putNumber(mTableName + " ty", getTy());
		SmartDashboard.putNumber(mTableName + " ta", getTa());
		SmartDashboard.putNumber(mTableName + " ts", getTs());
		SmartDashboard.putNumber(mTableName + " tl", getTl());
	}
}
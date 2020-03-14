package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.AftershockSubsystem;
import frc.lib.Limelight;
import frc.robot.Constants.LimelightConstants;
import frc.lib.Limelight.CameraMode;
import frc.lib.Limelight.LightMode;

/**
 * Container class for holding multiple Limelights
 * 
 * @author Shreyas Prasad
 */
public class LimelightManagerSubsystem extends AftershockSubsystem {

    private static LimelightManagerSubsystem mInstance;

    private final Limelight mShooterLimelight;
    private final Limelight mIntakeLimelight;
    private final List<Limelight> mLimelightList;

    /**
     * Constructor for LimelightManager Class
     */
    private LimelightManagerSubsystem() {
        super();
        
        mShooterLimelight = new Limelight(LimelightConstants.kShooterTableName);
        mIntakeLimelight = new Limelight(LimelightConstants.kIntakeTableName);

        mLimelightList = List.of(mShooterLimelight, mIntakeLimelight);
    }

    @Override
    public void initialize() {
        setAllLightMode(LightMode.eOn);
        setAllCameraMode(CameraMode.eVision);
    }

    /**
     * Gets Limelight 2+, mounted on the Shooter/Turret
     * 
     * @return Limelight for shooter targeting
     */
    public Limelight getShooterLimelight() {
        return mShooterLimelight;
    }

    /**
     * Gets Limelight 1, mounted on the Intake
     * 
     * @return Limelight for ball tracking
     */
    public Limelight getIntakeLimelight() {
        return mIntakeLimelight;
    }

    /**
     * Sets the Light Mode for both Limelights
     * 
     * @param mode the Light Mode to change all Limelights to
     */
    public void setAllLightMode(LightMode mode) {
        mLimelightList.forEach(limelight -> limelight.setLedMode(mode));
    }

    /**
     * Sets the Camera Mode for both Limelights
     * 
     * @param mode the Camera Mode to change all Limelights to
     */
    public void setAllCameraMode(CameraMode mode) {
        mLimelightList.forEach(limelight -> limelight.setCameraMode(mode));
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putData(getInstance());
        mLimelightList.forEach(limelight -> limelight.outputTelemetry());
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * @return LimelightManager Singleton Instance
     */
    public synchronized static LimelightManagerSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new LimelightManagerSubsystem();
        }
        return mInstance;
    }
}
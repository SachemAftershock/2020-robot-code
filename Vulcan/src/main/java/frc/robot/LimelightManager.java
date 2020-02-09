package frc.robot;

import frc.robot.Constants.LimelightConstants;

/**
 * Container class for holding Limelights
 * 
 * @author Shreyas Prasad
 */
public class LimelightManager {

    private static LimelightManager mInstance;

    private final Limelight mShooterLimelight;
    private final Limelight mIntakeLimelight;

    private LimelightManager() {
        mShooterLimelight = new Limelight(LimelightConstants.kShooterTableName);
        mIntakeLimelight = new Limelight(LimelightConstants.kIntakeTableName);
    }

    public Limelight getShooterLimelight() {
        return mShooterLimelight;
    }

    public Limelight getIntakeLimelight() {
        return mIntakeLimelight;
    }

    /**
     * @return LimelightManager Singleton Instance
     */
    public synchronized static LimelightManager getInstance() {
        if(mInstance == null) {
            mInstance = new LimelightManager();
        }
        return mInstance;
    }
}
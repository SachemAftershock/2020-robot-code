package frc.robot;

public final class Constants {
    //Throughout all code: All units are assumed to be in Inches, Seconds, and Degrees unless specified otherwise

    // XboxController Constants
    public static final int kControllerPrimaryId = 0;
    public static final int kControllerSecondaryId = 1;

    public static final double kDeadbandTolerance = 0.05;

    // Collision Avoidance Constants
    public static final int kCollisionUltrasonicId = 1;

    public static final double kUltrasonicValueToInches = 0.125;

    public static final int kDriveMotorPortAId = 1;
    public static final int kDriveMotorPortBId = 2;
    public static final int kDriveMotorPortCId = 3;
    public static final int kDriveMotorStarboardAId = 4;
    public static final int kDriveMotorStarboardBId = 5;
    public static final int kDriveMotorStarboardCId = 6;

    public static final int kGearShiftForwardId = 1;
    public static final int kGearShiftReverseId = 2;

    // Shooter Subsystem Constants
    public static final int kLauncherMotorId = 7;

    public static final int kLidarId = 0;

    public static final int kPidId = 0;

    public static final int kBallValveAForwardId = 3;
    public static final int kBallValveAReverseId = 4;

    public static final int kBallValveBForwardId = 5;
    public static final int kBallValveBReverseId = 6;

    
    // Turret Subsystem Constants
    public static final int kTurretMotorId = 9;

    public static final int kTurretEncoderDioId = 0;
    public static final int kTurretLimitSwitchId = 1;

    // Storage Subsystem Constants
    public static final int kFeederMotorId = 8;

    public static final int kChamberDetectorId = 0;
    public static final int kPreChamberDetectorId = 1;
    public static final int kIntakeDetectorId = 2;
    public static final int kEntryDetectorId = 3;
    
    // Wheel Controller Constants
    public static final int kWheelControllerId = 3;
    public static final int kUltrasonicId = 0;

    public static final int kExtenderForwardId = 7;
    public static final int kExtenderReverseId = 8;

    // Intake Constants
    public static final int kIntakeMotorId = 9;

    public static final int kIntakeExtenderForwardId = 0;
    public static final int kIntakeExtenderReverseId = 1; //PCM B    

    //Pneumatic Constants
    public static final int kPcmAId = 0;
    public static final int kPcmBId = 1;

}
package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    //Throughout all code: All units are assumed to be in Inches, Seconds, and Degrees
    //Unless otherwise specificed

    // XboxController Constants
    public static final int kControllerPrimaryId = 0;
    public static final int kControllerSecondaryId = 1;

    public static final double kDeadbandTolerance = 0.05;

    // Drivebase Subsystem Constants
    public static final int kDriveMotorPortAId = 1;
    public static final int kDriveMotorPortBId = 2;
    public static final int kDriveMotorPortCId = 3;
    public static final int kDriveMotorStarboardAId = 4;
    public static final int kDriveMotorStarboardBId = 5;
    public static final int kDriveMotorStarboardCId = 6;

    public static final int kGearShiftForwardId = 2;
    public static final int kGearShiftReverseId = 3;

    public static final double kWheelDiameter = 6;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kDriveEncoderPPR = 42.0;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelDiameter / 2.0);
    public static final double kTrackWidthMeters = Units.inchesToMeters(19.125); //Distance in between wheels

    // Shooter Subsystem Constants
    public static final int kLauncherMotorId = 7;
    public static final int kFeederMotorId = 8;

    public static final int kPidId = 0;

    public static final int kElevationForwardId = 4;
    public static final int kElevationReverseId = 5;
    
    // Turret Subsystem Constants
    public static final int kTurretMotorId = 9;

    public static final int kTurretEncoderDioId = 0;
    public static final int kTurretLimitSwitchId = 1;

    // Wheel Controller Constants
    public static final int kWheelControllerId = 3;
    public static final int kUltrasonicId = 0;

    public static final int kExtenderForwardId = 6;
    public static final int kExtenderReverseId = 7;

    // Intake Constants
    public static final int kIntakeMotorId = 9;

    public static final int kIntakeExtenderForwardId = 8;
    public static final int kIntakeExtenderReverseId = 9;
    

    //Pneumatic Constants
    public static final int kPcmId = 0;
}